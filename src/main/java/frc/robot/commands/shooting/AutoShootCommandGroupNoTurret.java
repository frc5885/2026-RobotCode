// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.LaunchCalculator;
import java.util.Optional;

/**
 * Auto shoot command group that aims the chassis instead of the turret. Overrides PathPlanner's
 * rotation target to point the robot at the launch target, allowing shoot-on-the-move in
 * autonomous.
 */
public class AutoShootCommandGroupNoTurret {
  public static Command create() {
    return Commands.parallel(new AimHoodAndSpinFlywheelCommand(), new ShootIfReadyCommand())
        .beforeStarting(
            () ->
                PPHolonomicDriveController.setRotationTargetOverride(
                    () ->
                        Optional.of(
                            LaunchCalculator.getInstance().getParameters().turretAngle())))
        .finallyDo(
            interrupted -> PPHolonomicDriveController.setRotationTargetOverride(null));
  }
}
