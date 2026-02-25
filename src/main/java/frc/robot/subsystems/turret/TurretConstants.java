// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TurretConstants {

  public static final double gearRatio = 10.0 / 1.0;
  public static final double minAngleRadians = 0.0;
  public static final double maxAngleRadians = 2 * Math.PI;
  public static final double startingAngleRadians = 0.0;
  public static final double momentOfInertia = 0.5; // kg*m^2
  public static final int canId = 27;
  public static final boolean motorInverted = false;
  public static final int currentLimit = 30;
  // Motor Rotations -> Radians
  public static final double positionConversionFactor = 2 * Math.PI / gearRatio;
  // Motor RPM -> Radians per second
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double kp = 5.0;
  public static final double ki = 0.0;
  public static final double kd = 3.0;

  public static final Transform3d robotToTurret =
      new Transform3d(-0.16, 0.16, 0.38, Rotation3d.kZero);
}
