// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Every number the students might want to change lives here, in one place.
 *
 * <p>Values that get tuned live on the dashboard (Kv, deadband, target) are NOT here -- those
 * are published to NetworkTables by the subsystems so they can be edited from Elastic without
 * redeploying. This file holds the fixed facts about the hardware (CAN IDs, gear ratios, travel
 * limits) and the safety limits that should never be exceeded.
 */
public final class Constants {
  private Constants() {}

  /**
   * Both Kraken X60s are on the CTRE CANivore, so every device is created on this bus.
   *
   * <p>The string is the CANivore's device name as shown in Phoenix Tuner X. If you renamed your
   * CANivore, change it here. Use "*" to grab the first CANivore the code finds.
   */
  public static final CANBus kCANBus = new CANBus("canivore");

  /** Mechanism A: the linear extension (Demo 1 -- bang-bang control). */
  public static final class LinearExtension {
    /** CAN device ID of this mechanism's Kraken X60 (set in Phoenix Tuner X). */
    public static final int kMotorCanId = 20;

    // --- Geometry: how motor rotations turn into CENTIMETERS of travel ---
    // 30-tooth : 12-tooth reduction = 2.5 motor rotations per pinion rotation. Written as the tooth
    // counts so it matches the physical gears.
    public static final double kGearRatio = 30.0 / 12.0; // motor rotations : pinion rotations (= 2.5)

    // Rack-and-pinion: the rack advances by the pinion's pitch circumference (pi * pitch diameter)
    // per pinion revolution. A 2.54 cm (1") pitch-diameter pinion gives ~7.98 cm per revolution.
    public static final double kPinionPitchDiameterCm = 2.54; // 1"
    public static final double kPinionCircumferenceCm = Math.PI * kPinionPitchDiameterCm;

    // TEMPORARY CALIBRATION -- set back to 1.0 once the real geometry is known.
    // A 1" pitch-diameter pinion at exactly 2.5:1 predicts 8.5" over the full stroke, but the
    // mechanism actually travels 11.5" -- a genuine, unexplained 1.35x factor. For a direct
    // rack-and-pinion the ONLY things that set the scale are the gear ratio and the pitch diameter,
    // so one of those isn't what we think (or there's an extra stage). The factor is dimensionless,
    // so it carries into any unit unchanged; it makes PositionCm and the soft limits read true
    // travel in the meantime. See README diagnostics.
    public static final double kPositionScaleCalibration = 11.5 / 8.5; // ~1.353

    // --- Travel limits (cm) --- 11.5" of travel = 29.21 cm.
    public static final double kMinCm = 0.0;
    public static final double kMaxCm = 29.21;

    // --- Motor configuration ---
    // If pressing "extend" (positive volts) actually retracts, flip this to Clockwise_Positive.
    public static final InvertedValue kInvert = InvertedValue.CounterClockwise_Positive;
    // Coast by default: when a controller commands 0 V (or in OFF / while disabled) the motor
    // freewheels, so students can slide the mechanism by hand to zero it. It will NOT hold position
    // on its own -- switch to NeutralModeValue.Brake if you want it to hold.
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final double kStatorCurrentLimitAmps = 40.0;
    public static final double kSupplyCurrentLimitAmps = 40.0;

    // --- Absolute voltage safety ceiling for BOTH controllers ---
    // No matter what a student types into Elastic (bang-bang "Kv" or the PID max voltage), the
    // applied voltage magnitude is clamped to this range. You chose 0-6 V for the workshop; raise
    // kMaxDriveVolts here if the mechanism needs more authority.
    public static final double kMinDriveVolts = 0.0;
    public static final double kMaxDriveVolts = 6.0;

    // --- Starting values shown on the dashboard (students edit these live) ---
    // Distance-based gains are per-CENTIMETER (1 V/in = 0.3937 V/cm). Converted to preserve the same
    // behavior -- re-tune as needed.
    // Demo 1 (bang-bang):
    public static final double kDefaultDriveVolts = 2.0; // the "Kv" property (volts)
    public static final double kDefaultDeadbandCm = 0.635; // 0.25"
    // Shared setpoint:
    public static final double kDefaultTargetCm = 10.16; // 4"
    // Demo 2 (PID). Start P-only (kI = kD = 0) and a gentle voltage cap.
    public static final double kDefaultKp = 0.3937; // volts per cm of error (= 1 V/in)
    public static final double kDefaultKi = 0.0;
    public static final double kDefaultKd = 0.0;
    public static final double kDefaultPidMaxVolts = 4.0; // clamped to kMaxDriveVolts above
    // Demo 3 (trapezoidal motion profile + feedforward). ks/kg/kv/ka turn the profiled velocity &
    // acceleration into voltage; the profile is bounded by a max velocity and acceleration. These
    // are STARTING POINTS -- characterize with SysId or hand-tune on the real mechanism.
    public static final double kDefaultKs = 0.1; // V to overcome static friction
    public static final double kDefaultKg = 0.0; // V to hold gravity; applied in EVERY enabled mode (0 if horizontal)
    public static final double kDefaultKv = 0.19685; // V per cm/s (= 0.5 V per in/s)
    public static final double kDefaultKa = 0.0; // V per cm/s^2
    public static final double kDefaultMaxVelCmPerSec = 25.4; // 10 in/s
    public static final double kDefaultMaxAccelCmPerSec2 = 50.8; // 20 in/s^2
    public static final double kDefaultProfileMaxVolts = 6.0; // output cap, clamped to kMaxDriveVolts
    // Path-following PID for Demo 3: corrects the error between the profile setpoint and the actual
    // position (NOT the final goal). The feedforward does the bulk, so these are usually small.
    public static final double kDefaultProfileKp = 0.3937; // volts per cm of PATH error (= 1 V/in)
    public static final double kDefaultProfileKi = 0.0;
    public static final double kDefaultProfileKd = 0.0;

    // Robot loop period; used for the motion-profile time step and feedforward discretization.
    public static final double kLoopPeriodSeconds = 0.020;

    // Settle timer: the mechanism counts as "arrived" when it is within this tolerance of the target
    // (cm) AND its speed is within this tolerance of zero (cm/s). One value is used for both.
    public static final double kDefaultSettleTolerance = 0.635; // 0.25"
  }

  /**
   * Mechanism B: the pivot. Has a simple PID and a motion-profile + ArmFeedforward + path-following
   * PID controller (mirroring the extension's Demos 2 and 3), adapted for an arm that fights gravity.
   */
  public static final class Pivot {
    /** CAN device ID of this mechanism's Kraken X60 (set in Phoenix Tuner X). */
    public static final int kMotorCanId = 21;

    // 72:14 gear stage, then two 5:1 stages -> ~128.57 motor rotations per arm rotation.
    public static final double kGearRatio = 72.0 / 14.0 * 5.0 * 5.0; // = 128.571...

    // Range of motion. Zero is horizontal; positive is counter-clockwise.
    public static final double kMinDegrees = 0.0;
    public static final double kMaxDegrees = 100.0;

    public static final InvertedValue kInvert = InvertedValue.CounterClockwise_Positive;
    // Coast by default (matches the extension). NOTE: the pivot fights gravity -- if a heavy arm
    // drifts down while disabled, switch this to NeutralModeValue.Brake.
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final double kStatorCurrentLimitAmps = 40.0;
    public static final double kSupplyCurrentLimitAmps = 40.0;

    // Voltage safety ceiling for every pivot controller.
    public static final double kMinDriveVolts = 0.0;
    public static final double kMaxDriveVolts = 6.0;

    // Shared setpoint (degrees, 0 = horizontal).
    public static final double kDefaultTargetDegrees = 45.0;

    // Simple PID (pure feedback, no gravity term -- it will sag under gravity, which is exactly what
    // motivates the feedforward controller below).
    public static final double kDefaultKp = 0.2; // volts per degree of error
    public static final double kDefaultKi = 0.0;
    public static final double kDefaultKd = 0.0;
    public static final double kDefaultPidMaxVolts = 6.0;

    // Motion profile + ArmFeedforward + path-following PID. Arm gravity varies with angle, so it is
    // kg*cos(angle-from-horizontal); the angle is fed to the feedforward in RADIANS for that cos
    // term, while velocity/acceleration stay in DEGREES -- so kv is V/(deg/s) and ka is V/(deg/s^2),
    // matching the profile's units. All STARTING POINTS -- characterize with SysId or hand-tune.
    public static final double kDefaultKs = 0.1; // V to overcome static friction
    public static final double kDefaultKg = 0.3; // V to hold the arm horizontal (MUST be tuned)
    public static final double kDefaultKv = 0.043; // V per deg/s (~280 deg/s free speed through 128.57:1)
    public static final double kDefaultKa = 0.0; // V per deg/s^2
    public static final double kDefaultMaxVelDegPerSec = 60.0;
    public static final double kDefaultMaxAccelDegPerSec2 = 120.0;
    public static final double kDefaultProfileMaxVolts = 6.0;
    public static final double kDefaultProfileKp = 0.1; // V per degree of PATH error
    public static final double kDefaultProfileKi = 0.0;
    public static final double kDefaultProfileKd = 0.0;

    // Robot loop period; used for the motion-profile step and feedforward discretization.
    public static final double kLoopPeriodSeconds = 0.020;

    // Settle timer: "arrived" when within this tolerance of the target (degrees) AND within this
    // tolerance of zero speed (deg/s). One value is used for both.
    public static final double kDefaultSettleTolerance = 1.0;
  }
}
