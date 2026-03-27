// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.leds.LEDConstants.LEDState;
import frc.robot.subsystems.leds.LEDSubsystem;

public class ShootCommandGroup extends ParallelCommandGroup {
  /** Creates a new ShootCommandGroup. */
  public ShootCommandGroup() {
    addCommands(
        new SlowDriveSpeedCommand(),
        new AimHoodAndSpinFlywheelCommand(),
        TurretCommands.setActiveLaunchingModeCommand(),
        new ShootIfReadyCommand(),
        LEDSubsystem.getInstance().applyState(LEDState.AIMING));
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
}
