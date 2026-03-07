// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SysIDCommands {
  private SysIDCommands() {}

  /*
   * Add all drive SysID commands to autochooser
   */
  public static void addDriveSysIdToAutoChooser(LoggedDashboardChooser<Command> autoChooser) {
    DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization());
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization());
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /*
   * Add all turret SysID commands to autochooser
   */
  public static void addTurretSysIdToAutoChooser(LoggedDashboardChooser<Command> autoChooser) {
    TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    autoChooser.addOption(
        "Turret SysId (Quasistatic Forward)",
        turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turret SysId (Quasistatic Reverse)",
        turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Turret SysId (Dynamic Forward)",
        turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turret SysId (Dynamic Reverse)",
        turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /*
   * Add all hood SysID commands to autochooser
   */
  public static void addHoodSysIdToAutoChooser(LoggedDashboardChooser<Command> autoChooser) {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    autoChooser.addOption(
        "Hood SysId (Quasistatic Forward)",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Hood SysId (Quasistatic Reverse)",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Hood SysId (Dynamic Forward)",
        shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Hood SysId (Dynamic Reverse)",
        shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
