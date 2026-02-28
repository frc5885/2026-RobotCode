// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
  public static final double gearRatio = 30.0 / 1.0;
  public static final double startingAngleRadians = Units.degreesToRadians(180.0);
  public static final double momentOfInertia = 0.0906; // kg*m^2, from CAD
  public static final int canId = 27;
  public static final boolean motorInverted = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double kp = 1.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.59607; // from sim
  public static final double kA = 0.125418; // from sim
  public static final double maxVelocityRadiansPerSecond = 10.0;
  public static final double maxAccelerationRadiansPerSecondSquared = 50.0;

  public static final double turretOffset = Units.degreesToRadians(180.0); // facing back
  private static final double minOffsetAngle = Units.degreesToRadians(-210.0);
  private static final double maxOffsetAngle = Units.degreesToRadians(210.0);
  public static final double minAngle = minOffsetAngle + turretOffset;
  public static final double maxAngle = maxOffsetAngle + turretOffset;
  public static final double trackOverlapMargin = Units.degreesToRadians(10);
  public static final double trackCenterRads = (minAngle + maxAngle) / 2;
  public static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  public static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  public static final Transform3d robotToTurret =
      new Transform3d(-0.16, 0.16, 0.38, new Rotation3d(0.0, 0.0, turretOffset));

  public static final double turretPositionToleranceRadians = Units.degreesToRadians(3.0);
  public static final double turretVelocityToleranceRadiansPerSecond = Units.degreesToRadians(5.0);
}
