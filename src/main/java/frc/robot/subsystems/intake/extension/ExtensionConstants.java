// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import edu.wpi.first.math.util.Units;

public class ExtensionConstants {
  public static final double gearRatio = 60.0 / 1.0;
  public static final double minAngleRadians = Units.degreesToRadians(7.0); // bottom
  public static final double maxAngleRadians = Units.degreesToRadians(117.17); // top
  public static final double startingAngleRadians =
      Units.degreesToRadians(
          117.17); // 90 degrees is defined as the forward linkage perpendicular to chassis

  // first number is position when down all the way, 2nd number is real angle when down all the way
  // set to zero, measure when down, only change first number
  public static final double absoluteEncoderOffset = 1.918 - 0.157;
  public static final double armLengthMeters = 0.2;
  public static final int leftCanId = 40;
  public static final int rightCanId = 41;
  public static final boolean leftMotorInverted = false;
  public static final boolean motorsOppositeDirections = false;
  public static final int currentLimit = 40;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;

  public static final double maxVelocityRadiansPerSecond = 5.3; // found both using advantagescope
  public static final double maxAccelerationRadiansPerSecondSquared = 80.0;

  public static final double kp = 20.0;
  public static final double ki = 0.0;
  public static final double kd = 0.2;
  // SysID Constants ⌄⌄⌄
  public static final double ks = 0.44952;
  public static final double kg = 0.21597 / 2.0;
  public static final double kv = 1.3555;
  public static final double ka = 0.14314;
  public static final double armOffsetToHorizontalRadians = -0.86084;
  // ^^^ SysID Constants
  public static final double positionToleranceRadians = Units.degreesToRadians(1.0);

  public static final double intakeStowedAngle = Units.degreesToRadians(116.0);
  public static final double intakeExtendedAngle =
      Units.degreesToRadians(-10.0); // elastics added more tension, need to compensate

  public static final double agitateTopAngle = 0.95;
  public static final double agitateBottomAngle = 0.6;
  public static final double agitateTimeSeconds = 0.5;

  public static final double intakeExtensionLengthMeters = Units.inchesToMeters(12.0);
}
