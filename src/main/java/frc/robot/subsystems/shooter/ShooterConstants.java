// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public static final double flywheelGearRatio = 10.0 / 1.0;
  public static final double flywheelMomentOfInertia = 0.1; // kg*m^2
  public static final int flywheelLeftCanId = 43;
  public static final int flywheelRightCanId = 44;
  public static final boolean flywheelLeftMotorInverted = false;
  public static final boolean flywheelMotorsOppositeDirections = false;
  public static final int flywheelCurrentLimit = 30;
  // Motor Rotations -> Flywheel Rotations
  public static final double flywheelPositionConversionFactor = 1 / flywheelGearRatio;
  // Motor RPM -> Flywheel RPM
  public static final double flywheelVelocityConversionFactor = flywheelPositionConversionFactor;

  public static final double hoodGearRatio = 10.0 / 1.0;
  public static final double hoodMinAngleRadians = 0.0;
  public static final double hoodMaxAngleRadians = 0.5;
  public static final double hoodStartingAngleRadians = 0.0;
  public static final double hoodArmLengthMeters = 1.0;
  public static final double hoodArmMassKG = 1.0;
  public static final int hoodCanId = 45;
  public static final boolean hoodMotorInverted = false;
  public static final int hoodCurrentLimit = 30;
  // Motor Rotations -> Radians
  public static final double hoodPositionConversionFactor = 2 * Math.PI / hoodGearRatio;
  // Motor RPM -> Radians per second
  public static final double hoodVelocityConversionFactor = hoodPositionConversionFactor / 60;
  public static final double hoodKp = 1.0;
  public static final double hoodKi = 0.0;
  public static final double hoodKd = 0.0;
}
