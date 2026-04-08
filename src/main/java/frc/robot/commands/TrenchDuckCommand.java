// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.turret.LaunchCalculator;
import frc.robot.util.OverrideUtil;
import frc.robot.util.Zones;
import org.littletonrobotics.junction.Logger;

/** Utility class to define a trigger for being in the trench duck zone. */
public class TrenchDuckCommand {
  /** Creates a new TrenchDuckCommand. */
  private TrenchDuckCommand() {}

  public static Trigger inTrenchDuckZoneTrigger() {
    DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    Trigger noOverridesActive =
        OverrideUtil.isManualModeTrigger()
            .negate()
            .and(() -> !DriverStation.isTest())
            .and(() -> !DriverStation.isAutonomous());

    Trigger inZone =
        Zones.trenchDuckZones
            .willContain(
                () -> LaunchCalculator.getTurretPosition(driveSubsystem.getPose()),
                () ->
                    LaunchCalculator.getFieldRelativeTurretVelocity(
                        driveSubsystem.getPose(), driveSubsystem.getFieldRelativeChassisSpeeds()),
                Seconds.of(DriveConstants.trenchDuckTimeSeconds))
            .debounce(0.1);

    // Workaround to log the value of inZone, AutoLogOutput doesn't work in here
    Trigger blankLoggingTrigger =
        new Trigger(
            () -> {
              Logger.recordOutput("Shooter/InTrenchDuckZone", inZone.getAsBoolean());
              return true;
            });
    return blankLoggingTrigger.and(noOverridesActive).and(inZone);
  }
}
