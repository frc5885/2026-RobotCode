// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.spindexer;

public class SpindexerConstants {
  public static final double gearRatio = 10.0 / 1.0;
  public static final double momentOfInertia = 0.01; // kg*m^2
  public static final int canId = 50;
  public static final boolean motorInverted = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Spindexer Rotations
  public static final double positionConversionFactor = 1 / gearRatio;
  // Motor RPM -> Spindexer RPM
  public static final double velocityConversionFactor = positionConversionFactor;
}
