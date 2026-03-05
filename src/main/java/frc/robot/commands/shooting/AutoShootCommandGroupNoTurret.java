// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.turret.LaunchCalculator;
import java.util.Optional;

/**
 * Auto shoot command group that aims the chassis instead of the turret. Overrides PathPlanner's
 * rotation target to point the robot at the launch target, allowing shoot-on-the-move in
 * autonomous.
 */
public class AutoShootCommandGroupNoTurret extends ParallelCommandGroup {
  public AutoShootCommandGroupNoTurret() {
    addCommands(new AimHoodAndSpinFlywheelCommand(), new ShootIfReadyCommand());
  }

  @Override
  public void initialize() {
    // Override PathPlanner's rotation target to aim at the launch target
    PPHolonomicDriveController.setRotationTargetOverride(
        () ->
            Optional.of(LaunchCalculator.getInstance().getParameters().turretAngle()));
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    // Clear the rotation override so PathPlanner resumes normal path rotation
    PPHolonomicDriveController.setRotationTargetOverride(null);
    super.end(interrupted);
  }
}
