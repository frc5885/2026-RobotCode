// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.turret.LaunchCalculator;

public class ShootCommandGroupNoTurret extends ParallelCommandGroup {
  /** Creates a new ShootCommandGroupNoTurret. */
  public ShootCommandGroupNoTurret(CommandXboxController controller) {
    addCommands(
        new SlowDriveSpeedCommand(),
        new AimHoodAndSpinFlywheelCommand(),
        DriveCommands.joystickDriveAtAngle(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> LaunchCalculator.getInstance().getParameters().turretAngle()),
        new ShootIfReadyCommand());
  }
}
