// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathUnderTrenchCommand extends Command {
  private Command cmd;
  private DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private DoubleSupplier leftJoystickSupplier;
  /** Creates a new PathUnderTrenchCommand. */
  public PathUnderTrenchCommand(DoubleSupplier leftJoystickSupplier) {
    this.leftJoystickSupplier = leftJoystickSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose = FieldConstants.getUnderTrenchTargetPose(driveSubsystem.getPose());
    cmd = AutoBuilder.pathfindToPose(targetPose, DriveConstants.pathConstraints, 0.0);
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
    cmd.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished() || leftJoystickSupplier.getAsDouble() > 0.1;
  }
}
