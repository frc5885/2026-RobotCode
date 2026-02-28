// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SlowDriveSpeedCommand extends Command {
  private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  /** Creates a new SlowDriveSpeedCommand. */
  public SlowDriveSpeedCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // We don't need drive subsystem as a requirement because this command doesn't actively
    // control it
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setDriveSpeedMultiplier(DriveConstants.shootOnTheMoveSpeedMultiplier);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setDriveSpeedMultiplier(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
