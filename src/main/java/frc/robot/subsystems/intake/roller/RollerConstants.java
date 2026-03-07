// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

public class RollerConstants {
  public static final double gearRatio = 10.0 / 1.0;
  public static final double momentOfInertia = 0.1; // kg*m^2
  public static final int leftCanId = 33;
  public static final int rightCanId = 34;
  public static final boolean leftMotorInverted = false;
  public static final boolean motorsOppositeDirections = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Intake Rotations
  public static final double positionConversionFactor = 1 / gearRatio;
  // Motor RPM -> Intake RPM
  public static final double velocityConversionFactor = positionConversionFactor;
}
