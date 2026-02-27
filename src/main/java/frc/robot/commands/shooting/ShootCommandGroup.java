// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShootCommandGroup extends ParallelCommandGroup {
  /** Creates a new ShootCommandGroup. */
  public ShootCommandGroup() {
    addCommands(
        new AimHoodAndSpinFlywheelCommand(),
        TurretCommands.setActiveLaunchingModeCommand(),
        new ShootIfReadyCommand());
  }
}
