// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.extension.ExtensionConstants;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.leds.LEDConstants.LEDState;
import frc.robot.subsystems.leds.LEDSubsystem;
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

  // For intake retract timeout
  private final Timer retractTimer = new Timer();
  private final double retractStallSeconds = 2.0;

  // LED Control
  private final Trigger intakeRunningTrigger =
      new Trigger(() -> currentState == IntakeState.INTAKING);

  public IntakeControlCommand(CommandXboxController controller) {
    this.controller = controller;
    addRequirements(intakeSubsystem);

    // Display intake LED state while intaking
    intakeRunningTrigger.whileTrue(LEDSubsystem.getInstance().applyState(LEDState.INTAKE_RUNNING));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = IntakeState.INITIAL;
    agitateTimer.stop();
    shootDelayTimer.stop();
    retractTimer.stop();
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
      retractTimer.stop();
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
          // unrelated but shouldn't this be stop intake?
          if (Constants.isSim()) intakeSubsystem.getIntakeSimulation().startIntake();
          break;

        case STOWED:
          retractTimer.reset();
          retractTimer.start();
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
      } else if (currentState == IntakeState.STOWED) {
        if (retractTimer.hasElapsed(retractStallSeconds)
            && !intakeSubsystem.isExtensionAtSetPoint()) {
          // If we've been trying to retract for a while and haven't succeeded, go to deployed.
          // Must call entry actions directly since mutating currentState here bypasses the
          // state-transition block, so the switch case won't run on the next cycle.
          currentState = IntakeState.DEPLOYED;
          retractTimer.stop();
          intakeSubsystem.setExtensionPosition(ExtensionConstants.intakeExtendedAngle);
          intakeSubsystem.setIntakeRollerVoltage(0);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeRollerVoltage(0);
    agitateTimer.stop();
    shootDelayTimer.stop();
    retractTimer.stop();
    currentState = IntakeState.INITIAL;
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
