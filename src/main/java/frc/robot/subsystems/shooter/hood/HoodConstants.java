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
  public static final double armLengthMeters = 0.1;
  public static final double armMassKG = 0.05;
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
}
