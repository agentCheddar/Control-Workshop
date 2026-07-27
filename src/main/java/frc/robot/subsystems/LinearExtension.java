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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Mechanism A -- a linear extension driven by one Kraken X60 through a 2.5:1 gearbox onto a 1"
 * spool, traveling 0" to 11.5".
 *
 * <p>This subsystem hosts TWO controllers the students compare, plus an OFF mode. Exactly one is
 * active at a time; the buttons on Elastic switch between them (see the command factories at the
 * bottom). Both controllers drive to the SAME target (the {@code Target_Inches} tunable).
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
 */
public class LinearExtension extends SubsystemBase {
  /** Which controller is currently driving the mechanism. Exactly one at a time. */
  public enum ControlMode {
    OFF,
    BANG_BANG,
    PID
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

  // Status signals we read each loop. Because we set SensorToMechanismRatio in the config below,
  // position/velocity come back in SPOOL rotations, not motor rotations.
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  // Raw MOTOR (rotor) rotations, before the gearbox. getPosition() above is already divided by the
  // gear ratio (SensorToMechanismRatio), so this is a separate signal -- handy for showing students
  // the 2.5:1 relationship between motor turns and spool travel.
  private final StatusSignal<Angle> rotorPositionSignal = motor.getRotorPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVoltageSignal = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();

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

  public LinearExtension() {
    motor.getConfigurator().apply(buildConfig());

    // We only read these signals; ask for them at 100 Hz and quiet the rest to keep CAN light.
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal);
    motor.optimizeBusUtilization();

    // Assume the mechanism boots fully retracted (0"). Students re-zero any time with the
    // Elastic "Zero Linear Extension" button (see RobotContainer).
    motor.setPosition(0.0);
  }

  private TalonFXConfiguration buildConfig() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted = Constants.LinearExtension.kInvert;
    cfg.MotorOutput.NeutralMode = Constants.LinearExtension.kNeutralMode;

    // Tell the TalonFX the gearbox ratio so it reports position/velocity at the SPOOL, not the
    // motor rotor. After this, getPosition() returns spool rotations.
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
        statorCurrentSignal);

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
    double outputVolts =
        switch (mode) {
          case BANG_BANG -> bangBangVolts(error);
          case PID -> pidVolts(positionInches, target);
          case OFF -> 0.0;
        };

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

    // Friendly text for an Elastic widget right next to the buttons.
    SmartDashboard.putString("LinearExtension/ActiveController", mode.toString());
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
      return 0.0; // close enough -> coast/brake, don't fight
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

  // ---- Controller selection. Each command switches to its mode and (because there is a single
  // mode field) automatically disables the others. Published as Elastic buttons in RobotContainer.
  // ignoringDisable(true) lets you pre-select a controller while the robot is still disabled. ----

  /** Sets the active controller and resets PID state when entering PID mode. */
  public void setControlMode(ControlMode newMode) {
    if (newMode == ControlMode.PID && mode != ControlMode.PID) {
      pid.reset(); // clear the integral accumulator so PID starts fresh
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

  // ---- Unit conversions (spool rotations <-> inches). The gear ratio is already handled by
  // SensorToMechanismRatio, so here we only convert spool rotations to travel. ----
  private static double rotationsToInches(double spoolRotations) {
    return spoolRotations * Constants.LinearExtension.kSpoolCircumferenceInches;
  }

  private static double inchesToRotations(double inches) {
    return inches / Constants.LinearExtension.kSpoolCircumferenceInches;
  }
}
