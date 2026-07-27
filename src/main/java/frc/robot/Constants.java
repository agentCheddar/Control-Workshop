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
    // 2.5 motor rotations spin the spool once; each spool rotation reels in one circumference.
    public static final double kGearRatio = 2.5; // motor rotations : spool rotations
    public static final double kSpoolDiameterInches = 1.0;
    public static final double kSpoolCircumferenceInches = Math.PI * kSpoolDiameterInches;

    // --- Travel limits (inches) ---
    public static final double kMinInches = 0.0;
    public static final double kMaxInches = 11.5;

    // --- Motor configuration ---
    // If pressing "extend" (positive volts) actually retracts, flip this to Clockwise_Positive.
    public static final InvertedValue kInvert = InvertedValue.CounterClockwise_Positive;
    // Brake so the mechanism holds its spot when the bang-bang controller commands 0 V.
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
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
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final double kStatorCurrentLimitAmps = 40.0;
    public static final double kSupplyCurrentLimitAmps = 40.0;

    // Safety clamp for any manual voltage nudges before a real controller exists.
    public static final double kMaxManualVolts = 4.0;
  }
}
