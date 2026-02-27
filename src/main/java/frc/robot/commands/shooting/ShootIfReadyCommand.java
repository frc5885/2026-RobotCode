// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.LaunchCalculator;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootIfReadyCommand extends Command {
  private final HopperSubsystem hopperSubsystem = HopperSubsystem.getInstance();

  private final double kickerVoltage = 12.0;
  private final double spindexerVoltage = 12.0;

  /** Creates a new ShootIfReadyCommand. */
  public ShootIfReadyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TurretSubsystem.getInstance().isAtGoal()
        && ShooterSubsystem.getInstance().isFlywheelAtSetpoint()
        && ShooterSubsystem.getInstance().isHoodAtSetpoint()
        && LaunchCalculator.getInstance().getParameters().isValid()) {
      hopperSubsystem.setKickerVoltage(kickerVoltage);
      hopperSubsystem.setSpindexerVoltage(spindexerVoltage);
      Logger.recordOutput("ShootIfReadyCommand/isReady", true);

      if (Constants.isSim()) {
        SimShotVisualizer.launchFuelWithRateLimit();
      }
    } else {
      hopperSubsystem.setKickerVoltage(0.0);
      hopperSubsystem.setSpindexerVoltage(0.0);
      Logger.recordOutput("ShootIfReadyCommand/isReady", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.setKickerVoltage(0.0);
    hopperSubsystem.setSpindexerVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
