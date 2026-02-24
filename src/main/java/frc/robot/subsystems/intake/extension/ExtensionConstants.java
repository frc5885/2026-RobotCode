// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

public class ExtensionConstants {
  public static final double gearRatio = 10.0 / 1.0;
  public static final double minAngleRadians = 0.0;
  public static final double maxAngleRadians = 0.5;
  public static final double startingAngleRadians = 0.0;
  public static final double armLengthMeters = 0.2;
  public static final double armMassKG = 0.1;
  public static final int leftCanId = 35;
  public static final int rightCanId = 36;
  public static final boolean leftMotorInverted = false;
  public static final boolean motorsOppositeDirections = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double kp = 1.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
}
