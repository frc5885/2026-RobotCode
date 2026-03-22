// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

public class RollerConstants {
  public static final double gearRatio = 40.0 / 15.0;
  public static final double momentOfInertia = 0.1; // kg*m^2
  public static final int leftCanId = 42;
  public static final int rightCanId = 43;
  public static final boolean leftMotorInverted = false;
  public static final boolean motorsOppositeDirections = true;
  public static final int currentLimit = 40;
  // Motor Rotations -> Intake Rotations
  public static final double positionConversionFactor = 1 / gearRatio;
  // Motor RPM -> Intake RPM
  public static final double velocityConversionFactor = positionConversionFactor;

  public static final double intakeRollerVoltage = 12.0;
  public static final double agitateRollerVoltage = 6.0;
}
