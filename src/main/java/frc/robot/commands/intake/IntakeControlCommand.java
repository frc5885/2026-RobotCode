// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.extension.ExtensionConstants;
import frc.robot.subsystems.intake.roller.RollerConstants;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeControlCommand extends Command {
  /** Creates a new IntakeControl. */
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  private final CommandXboxController controller;
  private IntakeState currentState = IntakeState.INITIAL;

  // For timed agitation sequence
  private final Timer agitateTimer = new Timer();
  private boolean agitateIsTop = true;

  // For shoot delay from non-agitating states
  private final Timer shootDelayTimer = new Timer();
  private final double shootDelayTimeSeconds = 2.0;

  public IntakeControlCommand(CommandXboxController controller) {
    this.controller = controller;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = IntakeState.INITIAL;
    agitateTimer.stop();
    shootDelayTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("IntakeState", currentState);

    boolean intakeHeld = controller.leftTrigger(0.1).getAsBoolean();
    boolean shootHeld = controller.rightTrigger(0.1).getAsBoolean();
    boolean stowPressed = controller.leftBumper().getAsBoolean();

    // Determine next state
    IntakeState newState;
    // intake overrides all
    if (intakeHeld) {
      newState = IntakeState.INTAKING;
    }
    // agitate if not intaking and shooting
    else if (shootHeld) {
      if (currentState == IntakeState.AGITATING || currentState == IntakeState.INTAKING) {
        newState = IntakeState.AGITATING;
      } else if (currentState == IntakeState.WAITING_TO_AGITATE) {
        newState =
            shootDelayTimer.hasElapsed(shootDelayTimeSeconds)
                ? IntakeState.AGITATING
                : IntakeState.WAITING_TO_AGITATE;
      } else if (currentState == IntakeState.DEPLOYED) {
        newState = IntakeState.WAITING_TO_AGITATE;
      } else {
        newState = IntakeState.AGITATING;
      }
    }
    // was intaking or already deployed: stay deployed
    else if (currentState == IntakeState.INTAKING || currentState == IntakeState.DEPLOYED) {
      newState = IntakeState.DEPLOYED;
    }
    // was agitating or waiting to agitate (shooting done): auto-stow
    else if (currentState == IntakeState.AGITATING
        || currentState == IntakeState.WAITING_TO_AGITATE) {
      newState = IntakeState.STOWED;
    } else {
      newState = currentState;
    }

    // stow button overrides from any state
    if (stowPressed) {
      newState = IntakeState.STOWED;
    }

    // Handle state entry side effects on transition
    if (newState != currentState) {
      // don't run periodically only on state change
      currentState = newState;
      agitateTimer.stop();
      shootDelayTimer.stop();
      switch (currentState) {
        case INTAKING:
          intakeSubsystem.setExtensionPosition(ExtensionConstants.intakeExtendedAngle);
          intakeSubsystem.setIntakeRollerVoltage(RollerConstants.intakeRollerVoltage);
          if (Constants.isSim()) intakeSubsystem.getIntakeSimulation().startIntake();
          break;

        case WAITING_TO_AGITATE:
          shootDelayTimer.reset();
          shootDelayTimer.start();
          break;

        case AGITATING:
          agitateTimer.reset();
          agitateTimer.start();
          agitateIsTop = true;
          intakeSubsystem.setIntakeRollerVoltage(RollerConstants.agitateRollerVoltage);
          intakeSubsystem.setExtensionPosition(ExtensionConstants.agitateTopAngle);
          break;

        case DEPLOYED:
          intakeSubsystem.setExtensionPosition(ExtensionConstants.intakeExtendedAngle);
          intakeSubsystem.setIntakeRollerVoltage(0);
          if (Constants.isSim()) intakeSubsystem.getIntakeSimulation().startIntake();
          break;

        case STOWED:
          intakeSubsystem.setExtensionPosition(ExtensionConstants.intakeStowedAngle);
          intakeSubsystem.setIntakeRollerVoltage(0);
          if (Constants.isSim()) intakeSubsystem.getIntakeSimulation().stopIntake();
          break;

        default:
          break;
      }
    }

    // the states are the same check if its agitating
    else {
      if (currentState == IntakeState.AGITATING) {
        if (agitateTimer.hasElapsed(ExtensionConstants.agitateTimeSeconds)) {
          agitateIsTop = !agitateIsTop;
          agitateTimer.reset();
          agitateTimer.start();
        }
        intakeSubsystem.setExtensionPosition(
            agitateIsTop
                ? ExtensionConstants.agitateTopAngle
                : ExtensionConstants.agitateBottomAngle);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeRollerVoltage(0);
    agitateTimer.stop();
    shootDelayTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum IntakeState {
    INITIAL,
    INTAKING,
    WAITING_TO_AGITATE,
    AGITATING,
    STOWED,
    DEPLOYED,
    OUTAKING
  }
}
