// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

public class TurretConstants {

  public static final double turretGearRatio = 10.0 / 1.0;
  public static final double turretMinAngleRadians = 0.0;
  public static final double turretMaxAngleRadians = 2 * Math.PI;
  public static final double turretStartingAngleRadians = 0.0;
  public static final double turretMomentOfInertia = 0.5; // kg*m^2
  public static final int turretCanId = 45;
  public static final boolean turretMotorInverted = false;
  public static final int turretCurrentLimit = 30;
  // Motor Rotations -> Radians
  public static final double turretPositionConversionFactor = 2 * Math.PI / turretGearRatio;
  // Motor RPM -> Radians per second
  public static final double turretVelocityConversionFactor = turretPositionConversionFactor / 60;
  public static final double turretKp = 5.0;
  public static final double turretKi = 0.0;
  public static final double turretKd = 3.0;
}
