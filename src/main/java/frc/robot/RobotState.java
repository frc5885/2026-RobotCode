package frc.robot;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotState {
  public static final ShooterSubsystem.ReadableState shooterState = ShooterSubsystem.getState();
}
