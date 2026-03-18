// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final double gearRatio = 1.0 / 1.0;
  public static final int leftCanId = 32;
  public static final int rightCanId = 33;
  public static final boolean leftMotorInverted = false;
  public static final boolean motorsOppositeDirections = true;
  public static final int currentLimit = 100;
  // Motor Rotations -> Flywheel Rotations
  public static final double positionConversionFactor = 1 / gearRatio;
  // Motor RPM -> Flywheel RPM
  public static final double velocityConversionFactor = positionConversionFactor;
  public static final double velocityToleranceRPM = 30.0;

  public static final double kp = 0.0005;
  public static final double ki = 0.0;
  public static final double kd = 0.008;
  public static final double ks = 0.11267;
  public static final double kv = 0.0018079;
  public static final double ka = 0.0033313;
}
