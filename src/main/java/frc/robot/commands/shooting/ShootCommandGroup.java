// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.leds.LEDConstants.LEDState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShootCommandGroup extends ParallelCommandGroup {
  /** Creates a new ShootCommandGroup. */
  public ShootCommandGroup() {
    addCommands(
        new SlowDriveSpeedCommand(),
        new AimHoodAndSpinFlywheelCommand(),
        TurretCommands.setActiveLaunchingModeCommand(),
        new ShootIfReadyCommand(),
        LEDSubsystem.getInstance().applyState(LEDState.AIMING),
        logErrors());
  }

  public Command conditionalShootPreload() {
    return new ConditionalCommand(
            this, Commands.none(), () -> SmartDashboard.getBoolean("ShootPreload", false))
        .withTimeout(1.8);
  }

  public Command withAgitation(double delaySeconds) {
    return new ParallelCommandGroup(
        this, new AgitateIntakeCommand().runRepeatedlyAndSpinRollerWithStartDelay(delaySeconds));
  }

  public Command logErrors() {
    double[] hoodErrorDegrees = {0.0};
    double[] flywheelErrorRPM = {0.0};
    double[] turretErrorDegrees = {0.0};
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    TurretSubsystem turret = TurretSubsystem.getInstance();
    return Commands.runEnd(
        () -> {
          hoodErrorDegrees[0] = Units.radiansToDegrees(shooter.getHoodErrorRadians());
          flywheelErrorRPM[0] = shooter.getFlywheelErrorRPM();
          turretErrorDegrees[0] = Units.radiansToDegrees(turret.getTurretErrorRadians());
          Logger.recordOutput("ShotTuning/AbsHoodErrorDegrees", hoodErrorDegrees[0]);
          Logger.recordOutput("ShotTuning/AbsFlywheelErrorRPM", flywheelErrorRPM[0]);
          Logger.recordOutput("ShotTuning/AbsTurretErrorDegrees", turretErrorDegrees[0]);
        },
        () -> {
          hoodErrorDegrees[0] = 0.0;
          flywheelErrorRPM[0] = 0.0;
          turretErrorDegrees[0] = 0.0;
          Logger.recordOutput("ShotTuning/AbsHoodErrorDegrees", hoodErrorDegrees[0]);
          Logger.recordOutput("ShotTuning/AbsFlywheelErrorRPM", flywheelErrorRPM[0]);
          Logger.recordOutput("ShotTuning/AbsTurretErrorDegrees", turretErrorDegrees[0]);
        });
  }
}
