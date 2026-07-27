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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Mechanism B -- a pivot driven by one Kraken X60 through a 150:1 gearbox. Zero is horizontal;
 * positive angles are counter-clockwise, over a range of 0deg to 100deg.
 *
 * <p>This class is deliberately just the "plumbing": it configures the motor, converts to degrees,
 * zeroes, logs telemetry, and offers a safety-clamped {@link #setVoltage(double)} so you can nudge
 * it by hand. The tuned controller (a later demo) will build on top of this. There is no controller
 * running yet, so with brake mode + a 150:1 ratio it simply holds position when left alone.
 */
public class Pivot extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.Pivot.kMotorCanId, Constants.kCANBus);
  // FOC disabled so behavior doesn't depend on Phoenix Pro licensing.
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(false);

  // With SensorToMechanismRatio set below, these read in ARM rotations, not motor rotations.
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  // Raw MOTOR (rotor) rotations, before the 150:1 gearbox. getPosition() above is already divided by
  // the gear ratio, so this is a separate signal.
  private final StatusSignal<Angle> rotorPositionSignal = motor.getRotorPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVoltageSignal = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrentSignal = motor.getStatorCurrent();

  public Pivot() {
    motor.getConfigurator().apply(buildConfig());
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal);
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

    // Soft-limit backstop so nobody drives it past its range while nudging it.
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToRotations(Constants.Pivot.kMaxDegrees);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToRotations(Constants.Pivot.kMinDegrees);

    return cfg;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal,
        rotorPositionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal);

    Logger.recordOutput("Pivot/PositionDegrees", getPositionDegrees()); // mechanism position
    Logger.recordOutput(
        "Pivot/MotorRotations", rotorPositionSignal.getValueAsDouble()); // motor rotation
    Logger.recordOutput("Pivot/VelocityDegreesPerSec", getVelocityDegreesPerSec());
    Logger.recordOutput("Pivot/AppliedVolts", appliedVoltageSignal.getValueAsDouble());
    Logger.recordOutput("Pivot/StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
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

  /**
   * Applies an open-loop voltage, clamped to a safe magnitude. Placeholder for manual testing until
   * a real controller is added in a later demo. The hardware soft limits still apply.
   */
  public void setVoltage(double volts) {
    double clamped =
        MathUtil.clamp(volts, -Constants.Pivot.kMaxManualVolts, Constants.Pivot.kMaxManualVolts);
    motor.setControl(voltageRequest.withOutput(clamped));
  }

  private static double rotationsToDegrees(double armRotations) {
    return armRotations * 360.0;
  }

  private static double degreesToRotations(double degrees) {
    return degrees / 360.0;
  }
}
