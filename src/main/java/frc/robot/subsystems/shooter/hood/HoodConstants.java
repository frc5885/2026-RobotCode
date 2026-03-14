// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final double gearRatio = 25.0 * 18.5 / 1.0;
  public static final double minAngleRadians = Units.degreesToRadians(45);
  public static final double maxAngleRadians = Units.degreesToRadians(80);
  public static final double startingAngleRadians = Units.degreesToRadians(80);
  public static final double idleAngleRadians = Units.degreesToRadians(80.0);
  // public static final double armOffsetToHorizontalRadians = -0.0042524; // from sysID
  public static final double armLengthMeters = 0.1;
  public static final double armMassKG = 0.05;
  public static final double maxVelocityRadiansPerSecond = 1.5;
  public static final double maxAccelerationRadiansPerSecondSquared = 8.0;
  public static final int canId = 31;
  public static final boolean motorInverted = false;
  public static final int currentLimit = 20;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double kp = 40.0;
  public static final double ki = 0.0;
  public static final double kd = 0.2;
  // Ran sysID in sim
  public static final double ks = 0.20006;
  // public static final double kg = 4.6706;
  public static final double kv = 3.9723;
  public static final double ka = 0.12179;
  public static final double positionToleranceRadians = Units.degreesToRadians(0.5);
  public static final double velocityToleranceRadiansPerSecond = Units.degreesToRadians(6.0);

  public static final double extraDuckDistance = Units.inchesToMeters(12.0);
}
