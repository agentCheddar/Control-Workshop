// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LinearExtension;
import frc.robot.subsystems.Pivot;

/**
 * Wires the robot together: creates the mechanisms and puts the "zero" buttons on the dashboard.
 *
 * <p>There are no joysticks in this project -- the linear extension follows a target that students
 * set from Elastic, so the whole demo is driven from the dashboard.
 */
public class RobotContainer {
  // Mechanism A (Demo 1). Its periodic() runs the bang-bang controller every loop.
  private final LinearExtension linearExtension = new LinearExtension();

  // Mechanism B. Scaffolded for a later demo; here it just reports telemetry and can be zeroed.
  private final Pivot pivot = new Pivot();

  public RobotContainer() {
    configureDashboardButtons();
  }

  /**
   * Publishes command buttons to NetworkTables. Elastic renders each of these as a clickable
   * button. {@code ignoringDisable(true)} lets students zero a mechanism while the robot is
   * DISABLED -- which is exactly when you want to zero (line the mechanism up by hand, then click).
   */
  private void configureDashboardButtons() {
    // Controller selector for the linear extension. Clicking one switches the active controller
    // and automatically disables the others. On boot nothing drives until you pick one.
    SmartDashboard.putData("LinearExtension/Use Bang-Bang", linearExtension.useBangBangCommand());
    SmartDashboard.putData("LinearExtension/Use PID", linearExtension.usePidCommand());
    SmartDashboard.putData(
        "LinearExtension/Use Motion Profile", linearExtension.useMotionProfileCommand());
    SmartDashboard.putData("LinearExtension/Disable Controller", linearExtension.disableCommand());

    // Zeroing buttons (define "here" as 0" / horizontal). Safe to click while disabled.
    SmartDashboard.putData(
        "LinearExtension/Zero",
        Commands.runOnce(linearExtension::zero, linearExtension)
            .ignoringDisable(true)
            .withName("Zero Linear Extension"));

    SmartDashboard.putData(
        "Pivot/Zero",
        Commands.runOnce(pivot::zero, pivot)
            .ignoringDisable(true)
            .withName("Zero Pivot"));
  }
}
