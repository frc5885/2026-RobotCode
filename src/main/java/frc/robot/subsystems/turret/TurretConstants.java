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
  public static final double gearRatio = 9.0 * 145.0 / 21.0;
  public static final double startingAngleRadians = Units.degreesToRadians(0.0);
  public static final int canId = 30;
  public static final boolean motorInverted = true; // CCW positive
  public static final int currentLimit = 40;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double kp = 24.0;
  public static final double ki = 0.0;
  public static final double kd = 0.0;
  public static final double kS = 0.17916;
  public static final double kV = 1.1061;
  public static final double kA = 0.017088;
  public static final double maxVelocityRadiansPerSecond = 9.8;
  public static final double maxAccelerationRadiansPerSecondSquared = 70.0;

  public static final double turretOffset = Units.degreesToRadians(0.0); // facing front
  private static final double minOffsetAngle = Units.degreesToRadians(-228.0);
  private static final double maxOffsetAngle = Units.degreesToRadians(195.0);
  public static final double minAngle = minOffsetAngle + turretOffset;
  public static final double maxAngle = maxOffsetAngle + turretOffset;
  public static final double trackOverlapMargin = Units.degreesToRadians(10);
  public static final double trackCenterRads = (minAngle + maxAngle) / 2;
  public static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  public static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  public static final Transform3d robotToTurret =
      new Transform3d(-0.16, 0.16, 0.38, new Rotation3d(0.0, 0.0, turretOffset));

  public static final double turretPositionToleranceRadians = Units.degreesToRadians(6.0);
  public static final double turretPassingToleranceRadians = Units.degreesToRadians(8.0);
  public static final double turretVelocityToleranceRadiansPerSecond =
      Units.degreesToRadians(100.0);

  public static final int absoluteEncoder1Port = 1;
  public static final int absoluteEncoder2Port = 2;
  public static final int absoluteEncoder1Teeth = 21;
  public static final int absoluteEncoder2Teeth = 22;
  public static final int bigGearTeeth = 145;

  // keep the negative sign
  public static final double absoluteEncoder1OffsetRotations = -0.6945;
  public static final double absoluteEncoder2OffsetRotations = -0.8438;
}
