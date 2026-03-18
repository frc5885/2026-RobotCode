// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.intake.MoveIntakeToPositionCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.extension.ExtensionConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AgitateIntakeCommand extends SequentialCommandGroup {
  /** Creates a new AgitateIntakeCommand. */
  public AgitateIntakeCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveIntakeToPositionCommand(ExtensionConstants.agitateTopAngle).withTimeout(1.0),
        new MoveIntakeToPositionCommand(ExtensionConstants.agitateBottomAngle).withTimeout(1.0));
  }

  public static Command runRepeatedlyAndSpinRoller() {
    return new AgitateIntakeCommand().repeatedly().alongWith(spinRollerCommand());
  }

  private static Command spinRollerCommand() {
    return new StartEndCommand(
        () -> IntakeSubsystem.getInstance().setIntakeRollerVoltage(6.0),
        () -> IntakeSubsystem.getInstance().setIntakeRollerVoltage(0.0));
  }
}
