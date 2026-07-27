// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Because we log with AdvantageKit, this extends {@link LoggedRobot} instead of the usual
 * {@code TimedRobot}. It behaves the same (a {@code periodic} method every 20 ms), but AdvantageKit
 * records every input and output so it can be replayed in AdvantageScope.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  public Robot() {
    // ---- AdvantageKit logging setup -- must run BEFORE anything else is created ----
    Logger.recordMetadata("ProjectName", "Control-Workshop");
    Logger.recordMetadata("Mechanisms", "A: LinearExtension (bang-bang), B: Pivot");

    if (isReal()) {
      // On the real robot: save a .wpilog (open it later in AdvantageScope) AND stream live.
      // WPILOGWriter with no argument writes to a FAT32 USB stick if one is plugged into the
      // roboRIO, otherwise to the roboRIO's own storage.
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // In simulation: just stream live to AdvantageScope / Elastic over NetworkTables.
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    // Build all subsystems and dashboard buttons.
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs the command scheduler every loop. This is what calls each subsystem's periodic()
    // method -- including the bang-bang controller in LinearExtension.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
