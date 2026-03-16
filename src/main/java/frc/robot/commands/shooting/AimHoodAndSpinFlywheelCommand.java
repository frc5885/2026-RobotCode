// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.turret.LaunchCalculator;
import frc.robot.subsystems.turret.LaunchCalculator.LaunchingParameters;
import frc.robot.util.TunableDouble;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimHoodAndSpinFlywheelCommand extends Command {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

  private final DoubleSupplier testModeHoodAngle =
      TunableDouble.register("Shooter/HoodAngle", Units.degreesToRadians(70.0));
  private final DoubleSupplier testFlywheelRPM =
      TunableDouble.register("Shooter/FlywheelRPM", 2500.0);
  /** Creates a new AimHoodAndSpinFlywheelCommand. */
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
    if (!DriverStation.isTest()) {
      LaunchingParameters launchingParameters = LaunchCalculator.getInstance().getParameters();
      shooterSubsystem.setHoodGoal(
          launchingParameters.hoodAngle(), launchingParameters.hoodVelocity());
      shooterSubsystem.setFlywheelVelocity(
          Units.radiansPerSecondToRotationsPerMinute(launchingParameters.flywheelSpeed()));
    } else {
      if (Constants.isTuningEnabled) {
        // Get these just so we can log them while testing
        LaunchCalculator.getInstance().getParameters();
      }
      shooterSubsystem.setHoodGoal(testModeHoodAngle.getAsDouble(), 0.0);
      shooterSubsystem.setFlywheelVelocity(testFlywheelRPM.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setHoodGoalPosition(HoodConstants.idleAngleRadians);
    shooterSubsystem.setFlywheelVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
