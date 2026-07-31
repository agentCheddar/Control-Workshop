// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Mechanism A -- a linear extension: one Kraken X60 through a 30:12 (2.5:1) gearbox to a 1"
 * pitch-diameter pinion driving a rack, traveling 0" to 11.5".
 *
 * <p>This subsystem hosts THREE controllers the students compare, plus an OFF mode. Exactly one is
 * active at a time; the buttons on Elastic switch between them (see the command factories at the
 * bottom). All controllers drive to the SAME target (the {@code Target_Inches} tunable).
 *
 * <h2>Demo 1: Bang-Bang</h2>
 *
 * Slams a fixed voltage (+/-Kv) toward the target, or 0 V once within the deadband. Simple, but it
 * can't ease in, so it overshoots and chatters.
 *
 * <h2>Demo 2: PID</h2>
 *
 * Output = kP*error + kI*(integral of error) + kD*(rate of error), clamped to a tunable max
 * voltage. Eases in smoothly as the error shrinks. Students tune kP, kI, kD live.
 *
 * <h2>Demo 3: Motion profile + feedforward + path-following PID</h2>
 *
 * A trapezoidal profile generates a smooth position/velocity setpoint that ramps up to a max
 * velocity and back down (bounded by a max acceleration). An elevator feedforward (ks, kg, kv, ka)
 * turns that profiled motion into voltage (predictive), and a PID corrects the error between the
 * PROFILE setpoint and the actual position -- so it tracks the whole path, not just the endpoint.
 * Students tune ks/kg/kv/ka, the max velocity/acceleration, the voltage cap, and the path-following
 * kP/kI/kD live; the profiled setpoint and path error are logged.
 */
public class LinearExtension extends SubsystemBase {
  /** Which controller is currently driving the mechanism. Exactly one at a time. */
  public enum ControlMode {
    OFF,
    BANG_BANG,
    PID,
    MOTION_PROFILE
  }

  private ControlMode mode = ControlMode.OFF; // safe default: nothing drives until a button is hit

  private final TalonFX motor =
      new TalonFX(Constants.LinearExtension.kMotorCanId, Constants.kCANBus);

  // Reuse ONE control-request object every loop (CTRE recommends this to avoid allocations).
  // FOC is disabled so behavior is identical whether or not the devices are Phoenix Pro
  // licensed -- keeps the workshop free of "not licensed" warnings.
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(false);

  // WPILib's PID controller does the math for Demo 2. Gains are pushed in from the dashboard each
  // loop, so students can tune live.
  private final PIDController pid =
      new PIDController(
          Constants.LinearExtension.kDefaultKp,
          Constants.LinearExtension.kDefaultKi,
          Constants.LinearExtension.kDefaultKd);

  // Demo 3: elevator feedforward (ks, kg, kv, ka) + a trapezoidal profile. Gains are pushed in live
  // via setters; the profile is rebuilt each loop from the tunable constraints. profileSetpoint is
  // the moving target the profile advances one loop at a time (dt = kLoopPeriodSeconds).
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          Constants.LinearExtension.kDefaultKs,
          Constants.LinearExtension.kDefaultKg,
          Constants.LinearExtension.kDefaultKv,
          Constants.LinearExtension.kDefaultKa,
          Constants.LinearExtension.kLoopPeriodSeconds);
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();
  // Path-following PID for Demo 3: feedback on the error between the profile setpoint and reality.
  // Separate from the Demo 2 `pid` so the two controllers tune independently.
  private final PIDController profilePid =
      new PIDController(
          Constants.LinearExtension.kDefaultProfileKp,
          Constants.LinearExtension.kDefaultProfileKi,
          Constants.LinearExtension.kDefaultProfileKd);

  // Status signals we read each loop. Because we set SensorToMechanismRatio in the config below,
  // position/velocity come back in PINION rotations, not motor rotations.
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  // Raw MOTOR (rotor) rotations, before the gearbox. getPosition() above is already divided by the
  // gear ratio (SensorToMechanismRatio), so this is a separate signal -- handy for showing students
  // the 30:12 (2.5:1) relationship between motor turns and rack travel.
  private final StatusSignal<Angle> rotorPositionSignal = motor.getRotorPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVoltageSignal = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrentSignal = motor.getSupplyCurrent();

  // ---- Dashboard tunables (published to NetworkTables under /Tuning, logged by AdvantageKit) ----
  // Anything under /Tuning shows up in AdvantageScope's tuning mode and can be edited in Elastic.
  private final LoggedNetworkNumber kvVolts =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Kv_Volts", Constants.LinearExtension.kDefaultDriveVolts);
  private final LoggedNetworkNumber deadbandInches =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Deadband_Inches",
          Constants.LinearExtension.kDefaultDeadbandInches);
  private final LoggedNetworkNumber targetInches =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Target_Inches", Constants.LinearExtension.kDefaultTargetInches);
  private final LoggedNetworkNumber kP =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kP", Constants.LinearExtension.kDefaultKp);
  private final LoggedNetworkNumber kI =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kI", Constants.LinearExtension.kDefaultKi);
  private final LoggedNetworkNumber kD =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kD", Constants.LinearExtension.kDefaultKd);
  private final LoggedNetworkNumber pidMaxVolts =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/PID_MaxVolts", Constants.LinearExtension.kDefaultPidMaxVolts);
  private final LoggedNetworkNumber kS =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kS", Constants.LinearExtension.kDefaultKs);
  private final LoggedNetworkNumber kG =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kG", Constants.LinearExtension.kDefaultKg);
  private final LoggedNetworkNumber kV =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kV", Constants.LinearExtension.kDefaultKv);
  private final LoggedNetworkNumber kA =
      new LoggedNetworkNumber("/Tuning/LinearExtension/kA", Constants.LinearExtension.kDefaultKa);
  private final LoggedNetworkNumber maxVelInPerSec =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Profile_MaxVel_InPerSec",
          Constants.LinearExtension.kDefaultMaxVelInPerSec);
  private final LoggedNetworkNumber maxAccelInPerSec2 =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Profile_MaxAccel_InPerSec2",
          Constants.LinearExtension.kDefaultMaxAccelInPerSec2);
  private final LoggedNetworkNumber profileMaxVolts =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Profile_MaxVolts",
          Constants.LinearExtension.kDefaultProfileMaxVolts);
  private final LoggedNetworkNumber profileKp =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Profile_kP", Constants.LinearExtension.kDefaultProfileKp);
  private final LoggedNetworkNumber profileKi =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Profile_kI", Constants.LinearExtension.kDefaultProfileKi);
  private final LoggedNetworkNumber profileKd =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Profile_kD", Constants.LinearExtension.kDefaultProfileKd);
  private final LoggedNetworkNumber settleTolerance =
      new LoggedNetworkNumber(
          "/Tuning/LinearExtension/Settle_Tolerance",
          Constants.LinearExtension.kDefaultSettleTolerance);

  // ---- Settle timer: measures how long the active controller takes to reach the target and stop,
  // so students can see how fast each controller really is. Starts when the target changes; stops
  // when |position error| and |velocity| are both within the tolerance. The average resets whenever
  // the robot is disabled. ----
  private final Timer settleTimer = new Timer();
  private double lastTarget = Double.NaN; // NaN so the very first loop isn't seen as a "change"
  private boolean timing = false;
  private double lastSettleSec = 0.0; // most recent completed settle time
  private double settleSumSec = 0.0; // running sum for the average (reset on disable)
  private int settleCount = 0;
  private boolean wasEnabled = false;

  public LinearExtension() {
    motor.getConfigurator().apply(buildConfig());

    // We only read these signals; ask for them at 100 Hz and quiet the rest to keep CAN light.
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal);
    motor.optimizeBusUtilization();

    // Assume the mechanism boots fully retracted (0"). Students re-zero any time with the
    // Elastic "Zero Linear Extension" button (see RobotContainer).
    motor.setPosition(0.0);
  }

  private TalonFXConfiguration buildConfig() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted = Constants.LinearExtension.kInvert;
    cfg.MotorOutput.NeutralMode = Constants.LinearExtension.kNeutralMode;

    // Tell the TalonFX the gearbox ratio so it reports position/velocity at the PINION, not the
    // motor rotor. After this, getPosition() returns pinion rotations.
    cfg.Feedback.SensorToMechanismRatio = Constants.LinearExtension.kGearRatio;

    cfg.CurrentLimits.StatorCurrentLimit = Constants.LinearExtension.kStatorCurrentLimitAmps;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.LinearExtension.kSupplyCurrentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Hardware soft limits are a safety backstop: even if a controller commands full voltage, the
    // TalonFX refuses to drive past the ends of travel. These depend on the zero being correct.
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        inchesToRotations(Constants.LinearExtension.kMaxInches);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        inchesToRotations(Constants.LinearExtension.kMinInches);

    return cfg;
  }

  @Override
  public void periodic() {
    // Pull the freshest sensor values off the CAN bus.
    BaseStatusSignal.refreshAll(
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal);

    double positionInches = getPositionInches();
    // Both controllers share this setpoint. Clamped to the physical range so a fat-fingered target
    // can't ask for the impossible.
    double target =
        MathUtil.clamp(
            targetInches.get(),
            Constants.LinearExtension.kMinInches,
            Constants.LinearExtension.kMaxInches);
    double error = target - positionInches; // + means we are below the target

    // Pick the active controller. Switching modes is what the Elastic buttons do.
    //
    // While the robot is DISABLED we run NO controller. periodic() keeps running when disabled, so
    // if we let the PID keep calling calculate() it would accumulate integral against an error the
    // motor can't act on -- winding up to its clamp and lurching the instant you enable. So we hold
    // 0 V and reset the stateful controllers (PID integrator, motion-profile setpoint) every
    // disabled loop, guaranteeing a fresh start on enable.
    double outputVolts;
    if (DriverStation.isEnabled()) {
      outputVolts =
          switch (mode) {
            case BANG_BANG -> bangBangVolts(error);
            case PID -> pidVolts(positionInches, target);
            case MOTION_PROFILE -> profileVolts(positionInches, target);
            case OFF -> 0.0;
          };
    } else {
      outputVolts = 0.0;
      pid.reset(); // no integral windup while disabled (Demo 2)
      profilePid.reset(); // ...and the Demo 3 path-following PID
      profileSetpoint = new TrapezoidProfile.State(positionInches, 0.0); // keep the profile at reality
    }

    motor.setControl(voltageRequest.withOutput(outputVolts));

    // ---- Common logging for AdvantageScope; controller-specific values are logged inside the
    // helper methods below. ----
    Logger.recordOutput("LinearExtension/ControlMode", mode.toString());
    Logger.recordOutput("LinearExtension/PositionInches", positionInches); // mechanism position
    Logger.recordOutput(
        "LinearExtension/MotorRotations", rotorPositionSignal.getValueAsDouble()); // motor rotation
    Logger.recordOutput("LinearExtension/TargetInches", target);
    Logger.recordOutput("LinearExtension/ErrorInches", error);
    Logger.recordOutput("LinearExtension/CommandedVolts", outputVolts);
    Logger.recordOutput("LinearExtension/AppliedVolts", appliedVoltageSignal.getValueAsDouble());
    Logger.recordOutput("LinearExtension/VelocityInchesPerSec", getVelocityInchesPerSec());
    Logger.recordOutput("LinearExtension/StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
    Logger.recordOutput("LinearExtension/SupplyCurrentAmps", supplyCurrentSignal.getValueAsDouble());

    // Friendly text for an Elastic widget right next to the buttons.
    SmartDashboard.putString("LinearExtension/ActiveController", mode.toString());

    // How fast did the active controller get there?
    updateSettleTimer(target, positionInches);
  }

  // ---- Settle timer: how fast did the active controller actually reach the target? ----
  private void updateSettleTimer(double target, double positionInches) {
    double tol = Math.abs(settleTolerance.get());
    double velocity = getVelocityInchesPerSec();
    boolean enabled = DriverStation.isEnabled();

    // The average resets each time the robot becomes disabled; also drop any move in progress.
    if (!enabled && wasEnabled) {
      settleSumSec = 0.0;
      settleCount = 0;
      timing = false;
      settleTimer.stop();
    }
    wasEnabled = enabled;

    // Only run the stopwatch while a real controller is actively driving.
    boolean active = enabled && mode != ControlMode.OFF;
    if (active) {
      // Start (restart) the stopwatch the instant a new target is commanded.
      if (!Double.isNaN(lastTarget) && Math.abs(target - lastTarget) > 1e-6) {
        settleTimer.restart();
        timing = true;
      }
      // Stop once we are within tolerance of the target AND essentially stopped.
      if (timing && Math.abs(target - positionInches) <= tol && Math.abs(velocity) <= tol) {
        lastSettleSec = settleTimer.get();
        settleTimer.stop();
        timing = false;
        settleSumSec += lastSettleSec;
        settleCount++;
      }
      lastTarget = target; // remember for next loop's change detection
    } else {
      timing = false; // not actively controlling -> no stopwatch
    }

    double averageSec = settleCount > 0 ? settleSumSec / settleCount : 0.0;

    Logger.recordOutput("LinearExtension/SettleTolerance", tol);
    Logger.recordOutput("LinearExtension/Timer_Running", timing);
    Logger.recordOutput(
        "LinearExtension/Timer_ElapsedSec", timing ? settleTimer.get() : lastSettleSec);
    Logger.recordOutput("LinearExtension/Timer_LastSettleSec", lastSettleSec);
    Logger.recordOutput("LinearExtension/Timer_AverageSettleSec", averageSec);
    Logger.recordOutput("LinearExtension/Timer_SettleCount", settleCount);
  }

  // ============================ DEMO 1: BANG-BANG ============================
  private double bangBangVolts(double error) {
    double kv =
        MathUtil.clamp(
            kvVolts.get(),
            Constants.LinearExtension.kMinDriveVolts,
            Constants.LinearExtension.kMaxDriveVolts);
    double deadband = Math.abs(deadbandInches.get());

    Logger.recordOutput("LinearExtension/ClampedKv", kv);
    Logger.recordOutput("LinearExtension/DeadbandInches", deadband);

    if (Math.abs(error) <= deadband) {
      return 0.0; // close enough -> 0 V, let it settle (coasts by default)
    }
    return error > 0.0 ? kv : -kv; // below target -> extend (+), above -> retract (-)
  }

  // ============================ DEMO 2: PID ============================
  private double pidVolts(double positionInches, double target) {
    // The max voltage is itself tunable, but never allowed past the hardware safety ceiling.
    double maxVolts =
        MathUtil.clamp(
            pidMaxVolts.get(),
            Constants.LinearExtension.kMinDriveVolts,
            Constants.LinearExtension.kMaxDriveVolts);

    // Push the live gains into the controller and keep the integral term from winding up past the
    // voltage cap.
    pid.setPID(kP.get(), kI.get(), kD.get());
    pid.setIntegratorRange(-maxVolts, maxVolts);

    double raw = pid.calculate(positionInches, target);
    double clamped = MathUtil.clamp(raw, -maxVolts, maxVolts);

    Logger.recordOutput("LinearExtension/PID_RawVolts", raw);
    Logger.recordOutput("LinearExtension/PID_MaxVolts", maxVolts);
    return clamped;
  }

  // ==================== DEMO 3: MOTION PROFILE + FEEDFORWARD + PATH-FOLLOWING PID ====================
  private double profileVolts(double positionInches, double target) {
    // Output voltage cap -- tunable, but never allowed past the hardware safety ceiling.
    double maxVolts =
        MathUtil.clamp(
            profileMaxVolts.get(),
            Constants.LinearExtension.kMinDriveVolts,
            Constants.LinearExtension.kMaxDriveVolts);

    // Live-tune the feedforward gains.
    feedforward.setKs(kS.get());
    feedforward.setKg(kG.get());
    feedforward.setKv(kV.get());
    feedforward.setKa(kA.get());

    // Live-tune the path-following PID; bound its integrator to the voltage cap to prevent windup.
    profilePid.setPID(profileKp.get(), profileKi.get(), profileKd.get());
    profilePid.setIntegratorRange(-maxVolts, maxVolts);

    // Rebuild the profile from the (live-tunable) constraints and step it one loop toward the goal.
    TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Math.abs(maxVelInPerSec.get()), Math.abs(maxAccelInPerSec2.get())));
    TrapezoidProfile.State goal = new TrapezoidProfile.State(target, 0.0);
    TrapezoidProfile.State next =
        profile.calculate(Constants.LinearExtension.kLoopPeriodSeconds, profileSetpoint, goal);

    // Feedforward: ks/kg hold it, kv for the profiled speed, ka for the profiled acceleration
    // (implied by the velocity change over one loop). This is the predictive part.
    double feedforwardVolts =
        feedforward.calculateWithVelocities(profileSetpoint.velocity, next.velocity);

    // Feedback on the PATH error: the reference is the profile's setpoint position for THIS step
    // (next.position), NOT the final goal. That's what makes the PID correct deviations along the
    // whole trajectory instead of only at the end.
    double feedbackVolts = profilePid.calculate(positionInches, next.position);

    double volts = feedforwardVolts + feedbackVolts;

    profileSetpoint = next; // advance the profile for the next loop

    // Log the TARGET PROFILE and the feedforward/feedback split. Overlay Profile_SetpointInches on
    // PositionInches to see tracking; Profile_PathErrorInches is what the PID is chewing on.
    Logger.recordOutput("LinearExtension/Profile_SetpointInches", next.position);
    Logger.recordOutput("LinearExtension/Profile_SetpointVelInPerSec", next.velocity);
    Logger.recordOutput("LinearExtension/Profile_GoalInches", target);
    Logger.recordOutput("LinearExtension/Profile_MaxVolts", maxVolts);
    Logger.recordOutput("LinearExtension/Profile_PathErrorInches", next.position - positionInches);
    Logger.recordOutput("LinearExtension/Profile_FeedforwardVolts", feedforwardVolts);
    Logger.recordOutput("LinearExtension/Profile_FeedbackVolts", feedbackVolts);

    return MathUtil.clamp(volts, -maxVolts, maxVolts);
  }

  // ---- Controller selection. Each command switches to its mode and (because there is a single
  // mode field) automatically disables the others. Published as Elastic buttons in RobotContainer.
  // ignoringDisable(true) lets you pre-select a controller while the robot is still disabled. ----

  /** Sets the active controller, resetting controller state on entry so it starts cleanly. */
  public void setControlMode(ControlMode newMode) {
    if (newMode == ControlMode.PID && mode != ControlMode.PID) {
      pid.reset(); // clear the integral accumulator so PID starts fresh
    }
    if (newMode == ControlMode.MOTION_PROFILE && mode != ControlMode.MOTION_PROFILE) {
      // Start the profile from where the mechanism actually is (assumed at rest) for a smooth
      // handoff -- otherwise it would jump from a stale setpoint.
      profileSetpoint = new TrapezoidProfile.State(getPositionInches(), 0.0);
      profilePid.reset(); // clear the path-following integrator so it starts fresh
    }
    mode = newMode;
  }

  public ControlMode getControlMode() {
    return mode;
  }

  public Command useBangBangCommand() {
    return runOnce(() -> setControlMode(ControlMode.BANG_BANG))
        .ignoringDisable(true)
        .withName("Use Bang-Bang");
  }

  public Command usePidCommand() {
    return runOnce(() -> setControlMode(ControlMode.PID))
        .ignoringDisable(true)
        .withName("Use PID");
  }

  public Command useMotionProfileCommand() {
    return runOnce(() -> setControlMode(ControlMode.MOTION_PROFILE))
        .ignoringDisable(true)
        .withName("Use Motion Profile");
  }

  public Command disableCommand() {
    return runOnce(() -> setControlMode(ControlMode.OFF))
        .ignoringDisable(true)
        .withName("Disable Controller");
  }

  /** Resets the encoder so the mechanism's current spot is treated as 0". */
  public void zero() {
    motor.setPosition(0.0);
  }

  /** Current extension in inches (0" = fully retracted). */
  public double getPositionInches() {
    return rotationsToInches(positionSignal.getValueAsDouble());
  }

  /** Current speed in inches per second (positive = extending). */
  public double getVelocityInchesPerSec() {
    return rotationsToInches(velocitySignal.getValueAsDouble());
  }

  // ---- Unit conversions (pinion rotations <-> inches of rack travel). The gear ratio is already
  // handled by SensorToMechanismRatio, so here we only convert pinion revolutions to travel. The
  // temporary scale calibration (see Constants.kPositionScaleCalibration) is folded in here. ----
  private static double rotationsToInches(double pinionRotations) {
    return pinionRotations
        * Constants.LinearExtension.kPinionCircumferenceInches
        * Constants.LinearExtension.kPositionScaleCalibration;
  }

  private static double inchesToRotations(double inches) {
    return inches
        / (Constants.LinearExtension.kPinionCircumferenceInches
            * Constants.LinearExtension.kPositionScaleCalibration);
  }
}
