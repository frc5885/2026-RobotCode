// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import edu.wpi.first.math.util.Units;

public class ExtensionConstants {
  public static final double gearRatio = 7.5 / 1.0;
  public static final double driveGearRadiusMeters = Units.inchesToMeters(0.9);
  public static final double minExtensionMeters = 0.0;
  public static final double maxExtensionMeters = 0.34;
  public static final double startingPositionMeters = 0.0;

  public static final int leftCanId = 40;
  public static final int rightCanId = 41;
  public static final boolean leftMotorInverted = false;
  public static final boolean rightMotorInverted = false;
  public static final int currentLimit = 20;
  // Motor Rotations -> Meters
  public static final double positionConversionFactor =
      2 * Math.PI * driveGearRadiusMeters / gearRatio;
  // Motor RPM -> Meters per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;

  public static final double kp = 10.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final double positionToleranceMeters = 0.02;

  /** Maximum allowed position difference between left and right before sync correction activates. */
  public static final double syncCorrectionThresholdMeters = 0.02;

  public static final double intakeStowedPosition = minExtensionMeters;
  public static final double intakeExtendedPosition = maxExtensionMeters;

  public static final double agitateFarPosition = 0.29;
  public static final double agitateNearPosition = 0.14;
  public static final double agitateTimeSeconds = 0.5;

  public static final double intakeExtensionLengthMeters = Units.inchesToMeters(12.0);
}
