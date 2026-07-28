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

    // --- Geometry: how motor rotations turn into inches of travel ---
    // 30-tooth : 12-tooth reduction = 2.5 motor rotations per pinion rotation. Written as the tooth
    // counts so it matches the physical gears.
    public static final double kGearRatio = 30.0 / 12.0; // motor rotations : pinion rotations (= 2.5)

    // Rack-and-pinion: the rack advances by the pinion's pitch circumference (pi * pitch diameter)
    // per pinion revolution. A 1" pitch-diameter pinion gives ~3.14" of travel per revolution.
    public static final double kPinionPitchDiameterInches = 1.0;
    public static final double kPinionCircumferenceInches = Math.PI * kPinionPitchDiameterInches;

    // TEMPORARY CALIBRATION -- set back to 1.0 once the real geometry is known.
    // A 1" pitch-diameter pinion at exactly 2.5:1 predicts 8.5" over the full stroke, but the
    // mechanism actually travels 11.5" -- a genuine, unexplained 1.35x factor. For a direct
    // rack-and-pinion the ONLY things that set the scale are the gear ratio and the pitch diameter,
    // so one of those isn't what we think (or there's an extra stage). This factor makes
    // PositionInches and the soft limits read true travel in the meantime. See README diagnostics.
    public static final double kPositionScaleCalibration = 11.5 / 8.5; // ~1.353

    // --- Travel limits (inches) ---
    public static final double kMinInches = 0.0;
    public static final double kMaxInches = 11.5;

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
    // Demo 1 (bang-bang):
    public static final double kDefaultDriveVolts = 2.0; // the "Kv" property
    public static final double kDefaultDeadbandInches = 0.25;
    // Shared setpoint:
    public static final double kDefaultTargetInches = 4.0;
    // Demo 2 (PID). Start P-only (kI = kD = 0) and a gentle voltage cap.
    public static final double kDefaultKp = 1.0; // volts per inch of error
    public static final double kDefaultKi = 0.0;
    public static final double kDefaultKd = 0.0;
    public static final double kDefaultPidMaxVolts = 4.0; // clamped to kMaxDriveVolts above
    // Demo 3 (trapezoidal motion profile + feedforward). ks/kg/kv/ka turn the profiled velocity &
    // acceleration into voltage; the profile is bounded by a max velocity and acceleration. These
    // are STARTING POINTS -- characterize with SysId or hand-tune on the real mechanism.
    public static final double kDefaultKs = 0.1; // V to overcome static friction
    public static final double kDefaultKg = 0.0; // V to hold against gravity (0 if horizontal)
    public static final double kDefaultKv = 0.5; // V per inch/sec
    public static final double kDefaultKa = 0.0; // V per inch/sec^2
    public static final double kDefaultMaxVelInPerSec = 10.0;
    public static final double kDefaultMaxAccelInPerSec2 = 20.0;

    // Robot loop period; used for the motion-profile time step and feedforward discretization.
    public static final double kLoopPeriodSeconds = 0.020;
  }

  /**
   * Mechanism B: the pivot. Scaffolded here (hardware config + telemetry + zeroing) so the wiring
   * is ready; its tuned controller is added in a later demo.
   */
  public static final class Pivot {
    /** CAN device ID of this mechanism's Kraken X60 (set in Phoenix Tuner X). */
    public static final int kMotorCanId = 21;

    // 150 motor rotations : 1 rotation of the pivot arm.
    public static final double kGearRatio = 150.0;

    // Range of motion. Zero is horizontal; positive is counter-clockwise.
    public static final double kMinDegrees = 0.0;
    public static final double kMaxDegrees = 100.0;

    public static final InvertedValue kInvert = InvertedValue.CounterClockwise_Positive;
    // Coast by default (matches the extension). NOTE: the pivot fights gravity -- if a heavy arm
    // drifts down while disabled, switch this to NeutralModeValue.Brake.
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final double kStatorCurrentLimitAmps = 40.0;
    public static final double kSupplyCurrentLimitAmps = 40.0;

    // Safety clamp for any manual voltage nudges before a real controller exists.
    public static final double kMaxManualVolts = 4.0;
  }
}
