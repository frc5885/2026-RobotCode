// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final double gearRatio = 10.0 / 1.0;
  public static final double minAngleRadians = Units.degreesToRadians(0);
  public static final double maxAngleRadians = Units.degreesToRadians(90);
  public static final double startingAngleRadians = Units.degreesToRadians(80);
  public static final double idleAngleRadians = Units.degreesToRadians(85.0);
  public static final double armOffsetToHorizontalRadians = -0.0042524; // from sysID
  public static final double armLengthMeters = 0.1;
  public static final double armMassKG = 0.05;
  public static final double maxVelocityRadiansPerSecond = Units.degreesToRadians(90);
  public static final double maxAccelerationRadiansPerSecondSquared = Units.degreesToRadians(360);
  public static final int canId = 45;
  public static final boolean motorInverted = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double kp = 1.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
  // Ran sysID in sim
  public static final double ks = 0.00010268;
  public static final double kg = 0.030181;
  public static final double kv = 0.10265;
  public static final double ka = 5.2044e-05;
  public static final double positionToleranceRadians = Units.degreesToRadians(1.0);
  public static final double velocityToleranceRadiansPerSecond = Units.degreesToRadians(6.0);

  public static final double extraDuckDistance = Units.inchesToMeters(12.0);
}
