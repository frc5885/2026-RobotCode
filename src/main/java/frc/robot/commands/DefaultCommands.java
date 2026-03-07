// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class DefaultCommands {
  private DefaultCommands() {}

  /*
   * Set up default drive command
   */
  public static void setDefaultDriveCommand(Command command) {
    DriveSubsystem.getInstance().setDefaultCommand(command);
  }

  /*
   * Set up default turret command
   */
  public static void setDefaultTurretCommand(Command command) {
    TurretSubsystem.getInstance().setDefaultCommand(command);
  }
}
