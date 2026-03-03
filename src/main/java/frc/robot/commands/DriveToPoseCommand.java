// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ChassisTrapezoidalController;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseCommand extends Command {
  private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private final Supplier<Pose2d> goalPoseSupplier;
  private final PIDController drivePIDController;
  private final PIDController turnPIDController;
  private final ChassisTrapezoidalController chassisController;

  /** Creates a new DriveToPoseCommand. */
  public DriveToPoseCommand(Supplier<Pose2d> goalPoseSupplier) {
    this.goalPoseSupplier = goalPoseSupplier;
    drivePIDController =
        new PIDController(
            DriveConstants.pathplannerDrivePID.kP,
            DriveConstants.pathplannerDrivePID.kI,
            DriveConstants.pathplannerDrivePID.kD);
    turnPIDController =
        new PIDController(
            DriveConstants.pathplannerTurnPID.kP,
            DriveConstants.pathplannerTurnPID.kI,
            DriveConstants.pathplannerTurnPID.kD);
    drivePIDController.setTolerance(DriveConstants.driveToPoseTranslationTolerance);
    turnPIDController.setTolerance(DriveConstants.driveToPoseRotationTolerance);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    chassisController =
        new ChassisTrapezoidalController(
            new TrapezoidProfile.Constraints(
                DriveConstants.pathConstraints.maxVelocityMPS(),
                DriveConstants.pathConstraints.maxAccelerationMPSSq()),
            new TrapezoidProfile.Constraints(
                DriveConstants.pathConstraints.maxAngularVelocityRadPerSec(),
                DriveConstants.pathConstraints.maxAngularAccelerationRadPerSecSq()),
            drivePIDController,
            turnPIDController);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisController.reset(
        driveSubsystem.getPose(), driveSubsystem.getChassisSpeeds(), goalPoseSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.runVelocity(chassisController.calculate(driveSubsystem.getPose()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassisController.isGoalAchieved();
  }
}
