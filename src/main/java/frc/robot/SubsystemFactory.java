// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.DriveSubsystem;

/** Factory for centralized subsystem initialization. */
public final class SubsystemFactory {
  private SubsystemFactory() {}

  public static void initAllSubsystems() {
    DriveSubsystem.getInstance();
  }
}
