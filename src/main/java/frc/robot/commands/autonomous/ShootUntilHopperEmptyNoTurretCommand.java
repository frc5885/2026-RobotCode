// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.shooting.AutoShootCommandGroupNoTurret;

/**
 * Auto shoot command that aims the chassis instead of the turret, running until the hopper is
 * empty. Overrides PathPlanner's path rotation to allow shoot-on-the-move in autonomous without a
 * turret.
 */
public class ShootUntilHopperEmptyNoTurretCommand extends ParallelDeadlineGroup {
  public ShootUntilHopperEmptyNoTurretCommand() {
    super(new WaitUntilHopperIsEmptyCommand());
    addCommands(new AutoShootCommandGroupNoTurret());
  }
}
