// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathUnderTrenchCommand extends Command {
  private Command cmd;
  private DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private BooleanSupplier leftJoystickSupplier;
  /** Creates a new PathUnderTrenchCommand. */
  public PathUnderTrenchCommand(BooleanSupplier leftJoystickMovedSupplier) {
    this.leftJoystickSupplier = leftJoystickMovedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pair<Pose2d, Pose2d> targetPoses =
        FieldConstants.getUnderTrenchTargetPoses(driveSubsystem.getPose());
    cmd =
        // AutoBuilder.pathfindToPose(targetPoses.getFirst(), DriveConstants.pathConstraints, 1.0)
        //     .andThen(
        //         AutoBuilder.pathfindToPose(
        //             targetPoses.getSecond(), DriveConstants.pathConstraints, 0.0));
        AutoBuilder.pathfindThenFollowPath(
            new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                    List.of(targetPoses.getFirst(), targetPoses.getSecond())),
                DriveConstants.pathConstraints,
                null,
                new GoalEndState(0, targetPoses.getSecond().getRotation())),
            DriveConstants.pathConstraints);
    cmd.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cmd.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmd.end(interrupted || !cmd.isFinished());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished() || leftJoystickSupplier.getAsBoolean();
  }
}
