// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Wrappers for TurretSubsystem commands. */
public class TurretCommands {
  /** Run the turret in tracking mode. Tracks the result of the launch calculator */
  public static Command runTrackTargetCommand() {
    return TurretSubsystem.getInstance().runTrackTargetCommand();
  }

  /** Lock to a supplied field-relative angle and velocity */
  public static Command runFieldRelativeFixedCommand(
      Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return TurretSubsystem.getInstance().runFieldRelativeFixedCommand(angle, velocity);
  }

  /**
   * Lock to a supplied robot-relative angle. Only should be used for testing or if vision is not
   * available
   */
  public static Command runRobotRelativeFixedCommand(Supplier<Rotation2d> angle) {
    return TurretSubsystem.getInstance().runRobotRelativeFixedCommand(angle);
  }
}
