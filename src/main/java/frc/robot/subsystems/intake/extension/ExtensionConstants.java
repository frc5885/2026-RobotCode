// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import edu.wpi.first.math.util.Units;

public class ExtensionConstants {
  public static final double gearRatio = 60.0 / 1.0;
  public static final double minAngleRadians = Units.degreesToRadians(10.0);
  public static final double maxAngleRadians = Units.degreesToRadians(116.0);
  public static final double startingAngleRadians = Units.degreesToRadians(116.0);
  public static final double armOffsetToHorizontalRadians = -1.5813;
  public static final double armLengthMeters = 0.2;
  public static final double armMassKG = 0.1;
  public static final int leftCanId = 40;
  public static final int rightCanId = 41;
  public static final boolean leftMotorInverted = false;
  public static final boolean motorsOppositeDirections = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;

  public static final double maxVelocityRadiansPerSecond = 8.0;
  public static final double maxAccelerationRadiansPerSecondSquared = 20.0;

  public static final double kp = 5.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
  public static final double ks = 0.82664;
  public static final double kg = 0.43804 / 1.2;
  public static final double kv = 0.66456 * 1.2;
  public static final double ka = 0.087336 * Math.pow(1.2, 2);
  public static final double positionToleranceRadians = Units.degreesToRadians(1.0);

  public static final double intakeStowedAngle = Units.degreesToRadians(115.0);
  public static final double intakeExtendedAngle = Units.degreesToRadians(11.0);

  public static final double intakeExtensionLengthMeters = Units.inchesToMeters(12.0);
}
