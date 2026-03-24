// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;

public class OverrideUtil {
  private static boolean manualMode = false;
  private static ShootingLocation shootingLocation = null;

  static {
    Logger.recordOutput("Overrides/ManualMode", manualMode);
    Logger.recordOutput("Overrides/ShootingLocation", shootingLocation);
  }

  public static void setManualMode(boolean manualMode) {
    OverrideUtil.manualMode = manualMode;
    Logger.recordOutput("Overrides/ManualMode", manualMode);
  }

  public static boolean isManualMode() {
    return manualMode;
  }

  public static void setShootingLocation(ShootingLocation shootingLocation) {
    OverrideUtil.shootingLocation = shootingLocation;
    Logger.recordOutput("Overrides/ShootingLocation", shootingLocation);
  }

  public static ShootingLocation getShootingLocation() {
    return shootingLocation;
  }

  public enum ShootingLocation {
    TOWER_FRONT_CENTER(new Pose2d()),
    OUTPOST_CORNER(new Pose2d()),
    RIGHT_WALL_CORNER(new Pose2d());

    public final Pose2d pose;

    ShootingLocation(Pose2d pose) {
      this.pose = pose;
    }
  }

  public static Command setManualModeCommand(boolean manualMode) {
    return new InstantCommand(() -> setManualMode(manualMode));
  }

  public static Command setShootingLocationCommand(ShootingLocation shootingLocation) {
    return new InstantCommand(() -> setShootingLocation(shootingLocation));
  }
}
