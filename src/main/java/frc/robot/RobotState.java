package frc.robot;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotState {
  public static ShooterSubsystem.ReadableState shooterSubsystemState = ShooterSubsystem.getState();
}
