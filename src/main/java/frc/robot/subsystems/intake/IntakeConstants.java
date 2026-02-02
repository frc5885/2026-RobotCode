// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final double intakeGearRatio = 10.0 / 1.0;
  public static final double intakeMomentOfInertia = 0.1; // kg*m^2
  public static final int intakeLeftCanId = 33;
  public static final int intakeRightCanId = 34;
  public static final boolean intakeLeftMotorInverted = false;
  public static final boolean intakeMotorsOppositeDirections = false;
  public static final int intakeCurrentLimit = 30;
  // Motor Rotations -> Intake Rotations
  public static final double intakePositionConversionFactor = 1 / intakeGearRatio;
  // Motor RPM -> Intake RPM
  public static final double intakeVelocityConversionFactor = intakePositionConversionFactor;

  public static final double extensionGearRatio = 10.0 / 1.0;
  public static final double extensionMinAngleRadians = 0.0;
  public static final double extensionMaxAngleRadians = 0.5;
  public static final double extensionStartingAngleRadians = 0.0;
  public static final double extensionArmLengthMeters = 1.0;
  public static final double extensionArmMassKG = 1.0;
  public static final int extensionLeftCanId = 35;
  public static final int extensionRightCanId = 36;
  public static final boolean extensionLeftMotorInverted = false;
  public static final boolean extensionMotorsOppositeDirections = false;
  public static final int extensionCurrentLimit = 30;
  // Motor Rotations -> Radians
  public static final double extensionPositionConversionFactor = 2 * Math.PI / extensionGearRatio;
  // Motor RPM -> Radians per second
  public static final double extensionVelocityConversionFactor =
      extensionPositionConversionFactor / 60;
  public static final double extensionKp = 1.0;
  public static final double extensionKi = 0.0;
  public static final double extensionKd = 0.0;
}
