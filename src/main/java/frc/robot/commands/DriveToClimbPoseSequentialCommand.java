// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FieldConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToClimbPoseSequentialCommand extends SequentialCommandGroup {

  Pose2d leftClimbApproachPose = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0));
  // new Pose2d(
  //     FieldConstants.Tower.centerPoint.getX(),
  //     FieldConstants.Tower.centerPoint.getY() + 1,
  //     Rotation2d.fromDegrees(0));
  Pose2d leftClimbPose =
      leftClimbApproachPose.transformBy(
          new Transform2d(new Translation2d(0, -0.5), Rotation2d.fromDegrees(0)));

  Pose2d rightClimbApproachPose = new Pose2d(1.0, 2.2, Rotation2d.fromDegrees(0));
  // new Pose2d(
  //     FieldConstants.Tower.centerPoint.getX(),
  //     FieldConstants.Tower.centerPoint.getY() + 1,
  //     Rotation2d.fromDegrees(0));
  Pose2d rightClimbPose =
      rightClimbApproachPose.transformBy(
          new Transform2d(new Translation2d(0, 0.5), Rotation2d.fromDegrees(0)));

  /** Creates a new DriveToClimbPoseSequentialCommand. */
  public DriveToClimbPoseSequentialCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveToPoseCommand(() -> getClimbApproachPose()), // .alongWith(new ClimbCommand()),
        new DriveToPoseCommand(() -> getClimbPose()));
  }

  private Pose2d getClimbApproachPose() {
    if (DriveSubsystem.getInstance().getPose().getY() >= FieldConstants.Tower.centerPoint.getY()) {
      return leftClimbApproachPose;
    } else {
      return rightClimbApproachPose;
    }
  }

  private Pose2d getClimbPose() {
    if (DriveSubsystem.getInstance().getPose().getY() >= FieldConstants.Tower.centerPoint.getY()) {
      return leftClimbPose;
    } else {
      return rightClimbPose;
    }
  }
}
