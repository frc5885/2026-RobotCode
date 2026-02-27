// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.turret.LaunchCalculator;
import frc.robot.subsystems.turret.LaunchCalculator.LaunchingParameters;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimHoodAndSpinFlywheelCommand extends Command {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  /** Creates a new AimHoodCommand. */
  public AimHoodAndSpinFlywheelCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LaunchingParameters launchingParameters = LaunchCalculator.getInstance().getParameters();
    shooterSubsystem.setHoodPosition(launchingParameters.hoodAngle());
    shooterSubsystem.setFlywheelVelocity(
        Units.radiansPerSecondToRotationsPerMinute(launchingParameters.flywheelSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setHoodPosition(HoodConstants.idleAngleRadians);
    shooterSubsystem.setFlywheelVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
