// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Factory for centralized subsystem initialization. */
public final class SubsystemFactory {
  private SubsystemFactory() {}

  public static void initAllSubsystems() {
    DriveSubsystem.getInstance();
    VisionSubsystem.getInstance();
    ShooterSubsystem.getInstance();
    IntakeSubsystem.getInstance();
  }
}
