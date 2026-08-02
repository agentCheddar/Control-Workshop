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
import edu.wpi.first.math.controller.ArmFeedforward;
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
 * Mechanism B -- a pivot driven by one Kraken X60 through a 72/14 * 5 * 5 (~128.57:1) gearbox. Zero is horizontal;
 * positive angles are counter-clockwise, over a range of 0deg to 100deg.
 *
 * <p>Mirrors the linear extension's Demo 2 and Demo 3, adapted for an arm:
 *
 * <ul>
 *   <li><b>PID</b> -- pure feedback to the target angle. No gravity term, so it will sag by a
 *       gravity-dependent amount; that steady-state error is the motivation for feedforward.
 *   <li><b>Motion profile + feedforward + PID</b> -- a trapezoidal profile plus an
 *       {@link ArmFeedforward}, whose gravity term is kg*cos(angle) so it compensates correctly at
 *       every angle (unlike the extension's constant kg), plus a PID on the PATH error.
 * </ul>
 *
 * Exactly one controller runs at a time; Elastic buttons switch between them. Both drive to the same
 * {@code Target_Degrees} tunable. On boot the controller is OFF.
 */
public class Pivot extends SubsystemBase {
  /** Which controller is currently driving the arm. Exactly one at a time. */
  public enum ControlMode {
    OFF,
    PID,
    MOTION_PROFILE
  }

  private ControlMode mode = ControlMode.OFF; // safe default: nothing drives until a button is hit

  private final TalonFX motor = new TalonFX(Constants.Pivot.kMotorCanId, Constants.kCANBus);
  // FOC disabled so behavior doesn't depend on Phoenix Pro licensing.
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(false);

  // Simple PID (pure feedback to the target angle).
  private final PIDController pid =
      new PIDController(
          Constants.Pivot.kDefaultKp, Constants.Pivot.kDefaultKi, Constants.Pivot.kDefaultKd);

  // Motion-profile controller: ArmFeedforward (gravity = kg*cos(angle)) + a path-following PID.
  // Gains are pushed in live via setters; the profile is rebuilt each loop from the tunable limits.
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.Pivot.kDefaultKs,
          Constants.Pivot.kDefaultKg,
          Constants.Pivot.kDefaultKv,
          Constants.Pivot.kDefaultKa,
          Constants.Pivot.kLoopPeriodSeconds);
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();
  private final PIDController profilePid =
      new PIDController(
          Constants.Pivot.kDefaultProfileKp,
          Constants.Pivot.kDefaultProfileKi,
          Constants.Pivot.kDefaultProfileKd);

  // With SensorToMechanismRatio set below, these read in ARM rotations, not motor rotations.
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  // Raw MOTOR (rotor) rotations, before the ~128.57:1 gearbox. getPosition() above is already divided by
  // the gear ratio, so this is a separate signal.
  private final StatusSignal<Angle> rotorPositionSignal = motor.getRotorPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVoltageSignal = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrentSignal = motor.getSupplyCurrent();

  // ---- Dashboard tunables (published under /Tuning, logged by AdvantageKit, editable in Elastic).
  private final LoggedNetworkNumber targetDegrees =
      new LoggedNetworkNumber(
          "/Tuning/Pivot/Target_Degrees", Constants.Pivot.kDefaultTargetDegrees);
  private final LoggedNetworkNumber kP =
      new LoggedNetworkNumber("/Tuning/Pivot/kP", Constants.Pivot.kDefaultKp);
  private final LoggedNetworkNumber kI =
      new LoggedNetworkNumber("/Tuning/Pivot/kI", Constants.Pivot.kDefaultKi);
  private final LoggedNetworkNumber kD =
      new LoggedNetworkNumber("/Tuning/Pivot/kD", Constants.Pivot.kDefaultKd);
  private final LoggedNetworkNumber pidMaxVolts =
      new LoggedNetworkNumber("/Tuning/Pivot/PID_MaxVolts", Constants.Pivot.kDefaultPidMaxVolts);
  private final LoggedNetworkNumber kS =
      new LoggedNetworkNumber("/Tuning/Pivot/kS", Constants.Pivot.kDefaultKs);
  private final LoggedNetworkNumber kG =
      new LoggedNetworkNumber("/Tuning/Pivot/kG", Constants.Pivot.kDefaultKg);
  private final LoggedNetworkNumber kV =
      new LoggedNetworkNumber("/Tuning/Pivot/kV", Constants.Pivot.kDefaultKv);
  private final LoggedNetworkNumber kA =
      new LoggedNetworkNumber("/Tuning/Pivot/kA", Constants.Pivot.kDefaultKa);
  private final LoggedNetworkNumber maxVelDegPerSec =
      new LoggedNetworkNumber(
          "/Tuning/Pivot/Profile_MaxVel_DegPerSec", Constants.Pivot.kDefaultMaxVelDegPerSec);
  private final LoggedNetworkNumber maxAccelDegPerSec2 =
      new LoggedNetworkNumber(
          "/Tuning/Pivot/Profile_MaxAccel_DegPerSec2", Constants.Pivot.kDefaultMaxAccelDegPerSec2);
  private final LoggedNetworkNumber profileMaxVolts =
      new LoggedNetworkNumber(
          "/Tuning/Pivot/Profile_MaxVolts", Constants.Pivot.kDefaultProfileMaxVolts);
  private final LoggedNetworkNumber profileKp =
      new LoggedNetworkNumber("/Tuning/Pivot/Profile_kP", Constants.Pivot.kDefaultProfileKp);
  private final LoggedNetworkNumber profileKi =
      new LoggedNetworkNumber("/Tuning/Pivot/Profile_kI", Constants.Pivot.kDefaultProfileKi);
  private final LoggedNetworkNumber profileKd =
      new LoggedNetworkNumber("/Tuning/Pivot/Profile_kD", Constants.Pivot.kDefaultProfileKd);
  private final LoggedNetworkNumber settleTolerance =
      new LoggedNetworkNumber(
          "/Tuning/Pivot/Settle_Tolerance", Constants.Pivot.kDefaultSettleTolerance);

  // ---- Settle timer: how long the active controller takes to reach the target angle and stop.
  // Starts when the target changes; stops when |angle error| and |speed| are both within the
  // tolerance. The average resets whenever the robot is disabled. ----
  private final Timer settleTimer = new Timer();
  private double lastTarget = Double.NaN; // NaN so the very first loop isn't seen as a "change"
  private boolean timing = false;
  private double lastSettleSec = 0.0;
  private double settleSumSec = 0.0;
  private int settleCount = 0;
  private boolean wasEnabled = false;

  public Pivot() {
    motor.getConfigurator().apply(buildConfig());
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal);
    motor.optimizeBusUtilization();

    // Assume we boot at horizontal (0deg). Re-zero with the Elastic "Zero Pivot" button.
    motor.setPosition(0.0);
  }

  private TalonFXConfiguration buildConfig() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted = Constants.Pivot.kInvert;
    cfg.MotorOutput.NeutralMode = Constants.Pivot.kNeutralMode;

    // Report position/velocity at the arm, not the rotor.
    cfg.Feedback.SensorToMechanismRatio = Constants.Pivot.kGearRatio;

    cfg.CurrentLimits.StatorCurrentLimit = Constants.Pivot.kStatorCurrentLimitAmps;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.Pivot.kSupplyCurrentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Soft-limit backstop so no controller can drive past the arm's range of motion.
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        degreesToRotations(Constants.Pivot.kMaxDegrees);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        degreesToRotations(Constants.Pivot.kMinDegrees);

    return cfg;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal);

    double positionDegrees = getPositionDegrees();
    double target =
        MathUtil.clamp(
            targetDegrees.get(), Constants.Pivot.kMinDegrees, Constants.Pivot.kMaxDegrees);
    double error = target - positionDegrees;

    // No controller runs while disabled -- otherwise the PID integrators would wind up and the
    // profile setpoint would go stale, lurching the arm on enable. Reset that state each loop.
    double outputVolts;
    if (DriverStation.isEnabled()) {
      outputVolts =
          switch (mode) {
            case PID -> pidVolts(positionDegrees, target);
            case MOTION_PROFILE -> profileVolts(positionDegrees, target);
            case OFF -> 0.0;
          };
    } else {
      outputVolts = 0.0;
      pid.reset();
      profilePid.reset();
      profileSetpoint = new TrapezoidProfile.State(positionDegrees, 0.0);
    }

    motor.setControl(voltageRequest.withOutput(outputVolts));

    Logger.recordOutput("Pivot/ControlMode", mode.toString());
    Logger.recordOutput("Pivot/PositionDegrees", positionDegrees); // mechanism position
    Logger.recordOutput(
        "Pivot/MotorRotations", rotorPositionSignal.getValueAsDouble()); // motor rotation
    Logger.recordOutput("Pivot/TargetDegrees", target);
    Logger.recordOutput("Pivot/ErrorDegrees", error);
    Logger.recordOutput("Pivot/CommandedVolts", outputVolts);
    Logger.recordOutput("Pivot/AppliedVolts", appliedVoltageSignal.getValueAsDouble());
    Logger.recordOutput("Pivot/VelocityDegreesPerSec", getVelocityDegreesPerSec());
    Logger.recordOutput("Pivot/StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
    Logger.recordOutput("Pivot/SupplyCurrentAmps", supplyCurrentSignal.getValueAsDouble());

    SmartDashboard.putString("Pivot/ActiveController", mode.toString());

    // How fast did the active controller get there?
    updateSettleTimer(target, positionDegrees);
  }

  // ---- Settle timer: how fast did the active controller actually reach the target angle? ----
  private void updateSettleTimer(double target, double positionDegrees) {
    double tol = Math.abs(settleTolerance.get());
    double velocity = getVelocityDegreesPerSec();
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
      if (timing && Math.abs(target - positionDegrees) <= tol && Math.abs(velocity) <= tol) {
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

    Logger.recordOutput("Pivot/SettleTolerance", tol);
    Logger.recordOutput("Pivot/Timer_Running", timing);
    Logger.recordOutput("Pivot/Timer_ElapsedSec", timing ? settleTimer.get() : lastSettleSec);
    Logger.recordOutput("Pivot/Timer_LastSettleSec", lastSettleSec);
    Logger.recordOutput("Pivot/Timer_AverageSettleSec", averageSec);
    Logger.recordOutput("Pivot/Timer_SettleCount", settleCount);
  }

  // ============================ SIMPLE PID (pure feedback) ============================
  private double pidVolts(double positionDegrees, double target) {
    double maxVolts =
        MathUtil.clamp(
            pidMaxVolts.get(), Constants.Pivot.kMinDriveVolts, Constants.Pivot.kMaxDriveVolts);

    pid.setPID(kP.get(), kI.get(), kD.get());
    pid.setIntegratorRange(-maxVolts, maxVolts);

    double raw = pid.calculate(positionDegrees, target);
    double clamped = MathUtil.clamp(raw, -maxVolts, maxVolts);

    Logger.recordOutput("Pivot/PID_RawVolts", raw);
    Logger.recordOutput("Pivot/PID_MaxVolts", maxVolts);
    return clamped;
  }

  // =============== MOTION PROFILE + ARM FEEDFORWARD + PATH-FOLLOWING PID ===============
  private double profileVolts(double positionDegrees, double target) {
    double maxVolts =
        MathUtil.clamp(
            profileMaxVolts.get(), Constants.Pivot.kMinDriveVolts, Constants.Pivot.kMaxDriveVolts);

    // Live-tune the feedforward gains and the path-following PID.
    feedforward.setKs(kS.get());
    feedforward.setKg(kG.get());
    feedforward.setKv(kV.get());
    feedforward.setKa(kA.get());
    profilePid.setPID(profileKp.get(), profileKi.get(), profileKd.get());
    profilePid.setIntegratorRange(-maxVolts, maxVolts);

    // Step the trapezoidal profile (in degrees) one loop toward the goal.
    TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Math.abs(maxVelDegPerSec.get()), Math.abs(maxAccelDegPerSec2.get())));
    TrapezoidProfile.State goal = new TrapezoidProfile.State(target, 0.0);
    TrapezoidProfile.State next =
        profile.calculate(Constants.Pivot.kLoopPeriodSeconds, profileSetpoint, goal);

    // Feedforward. ArmFeedforward wants the angle in RADIANS from horizontal for the gravity
    // (kg*cos) term; velocities stay in deg/s, so kv/ka are in per-degree units to match the profile.
    double feedforwardVolts =
        feedforward.calculateWithVelocities(
            Math.toRadians(profileSetpoint.position), profileSetpoint.velocity, next.velocity);

    // Feedback on the PATH error: reference is the profile setpoint for this step, not the goal.
    double feedbackVolts = profilePid.calculate(positionDegrees, next.position);

    double volts = feedforwardVolts + feedbackVolts;

    profileSetpoint = next; // advance for the next loop

    Logger.recordOutput("Pivot/Profile_SetpointDegrees", next.position);
    Logger.recordOutput("Pivot/Profile_SetpointVelDegPerSec", next.velocity);
    Logger.recordOutput("Pivot/Profile_GoalDegrees", target);
    Logger.recordOutput("Pivot/Profile_MaxVolts", maxVolts);
    Logger.recordOutput("Pivot/Profile_PathErrorDegrees", next.position - positionDegrees);
    Logger.recordOutput("Pivot/Profile_FeedforwardVolts", feedforwardVolts);
    Logger.recordOutput("Pivot/Profile_FeedbackVolts", feedbackVolts);

    return MathUtil.clamp(volts, -maxVolts, maxVolts);
  }

  // ---- Controller selection. Each command switches to its mode and disables the others. Published
  // as Elastic buttons in RobotContainer. ignoringDisable(true) lets you pre-select while disabled.

  /** Sets the active controller, resetting controller state on entry so it starts cleanly. */
  public void setControlMode(ControlMode newMode) {
    if (newMode == ControlMode.PID && mode != ControlMode.PID) {
      pid.reset();
    }
    if (newMode == ControlMode.MOTION_PROFILE && mode != ControlMode.MOTION_PROFILE) {
      profileSetpoint = new TrapezoidProfile.State(getPositionDegrees(), 0.0);
      profilePid.reset();
    }
    mode = newMode;
  }

  public ControlMode getControlMode() {
    return mode;
  }

  public Command usePidCommand() {
    return runOnce(() -> setControlMode(ControlMode.PID)).ignoringDisable(true).withName("Use PID");
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

  /** Resets the encoder so the pivot's current spot is treated as horizontal (0deg). */
  public void zero() {
    motor.setPosition(0.0);
  }

  /** Current pivot angle in degrees (0 = horizontal, + = counter-clockwise). */
  public double getPositionDegrees() {
    return rotationsToDegrees(positionSignal.getValueAsDouble());
  }

  /** Current pivot speed in degrees per second. */
  public double getVelocityDegreesPerSec() {
    return rotationsToDegrees(velocitySignal.getValueAsDouble());
  }

  private static double rotationsToDegrees(double armRotations) {
    return armRotations * 360.0;
  }

  private static double degreesToRotations(double degrees) {
    return degrees / 360.0;
  }
}
