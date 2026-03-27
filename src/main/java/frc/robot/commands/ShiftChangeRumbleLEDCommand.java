// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.leds.LEDConstants.LEDState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.util.ControllerUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShiftChangeRumbleLEDCommand extends ParallelDeadlineGroup {
  /** Creates a new ShiftChangeRumbleLEDCommand. */
  public ShiftChangeRumbleLEDCommand(CommandXboxController controller, double duration) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(duration));
    addCommands(
        ControllerUtil.rumble(controller),
        LEDSubsystem.getInstance().applyState(LEDState.SHIFT_CHANGE));
  }

  /** Schedules the command to run */
  @Override
  public void schedule() {
    CommandScheduler.getInstance().schedule(this);
  }
}
