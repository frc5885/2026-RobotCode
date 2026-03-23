// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Helper functions for Geometry */
public class GeometryUtil {
  public static Rotation2d getNearest90Rotation(Rotation2d angle) {
    return Rotation2d.fromDegrees(Math.round(angle.getDegrees() / 90) * 90);
  }

  public static Rotation2d getNearest180Rotation(Rotation2d angle) {
    return Rotation2d.fromDegrees(Math.round(angle.getDegrees() / 180) * 180);
  }

  public static Rotation2d getNearest90or270Rotation(Rotation2d angle) {
    return Rotation2d.fromDegrees(Math.round((angle.getDegrees() - 90.0) / 180.0) * 180.0 + 90.0);
  }

  /** Returns the candidate from {@code candidates} nearest to {@code angle}. */
  public static Rotation2d getNearestRotation(Rotation2d angle, Rotation2d... candidates) {
    if (candidates == null || candidates.length == 0) {
      throw new IllegalArgumentException("getNearestRotation requires at least one candidate");
    }
    Rotation2d nearest = candidates[0];
    double minDist = Double.MAX_VALUE;
    for (Rotation2d candidate : candidates) {
      double dist =
          Math.abs(MathUtil.inputModulus(angle.getDegrees() - candidate.getDegrees(), -180, 180));
      if (dist < minDist) {
        minDist = dist;
        nearest = candidate;
      }
    }
    return nearest;
  }
}
