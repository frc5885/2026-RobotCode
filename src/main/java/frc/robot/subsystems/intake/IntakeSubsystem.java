// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem INSTANCE = null;

  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new IntakeSubsystem(new IntakeIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new IntakeSubsystem(new IntakeIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new IntakeSubsystem(new IntakeIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert intakeLeftMotorDisconnectedAlert;
  private final Alert intakeRightMotorDisconnectedAlert;
  private final Alert extensionLeftMotorDisconnectedAlert;
  private final Alert extensionRightMotorDisconnectedAlert;
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final PIDController extensionPID =
      new PIDController(
          IntakeConstants.extensionKp, IntakeConstants.extensionKi, IntakeConstants.extensionKd);

  /** Creates a new Intake. */
  private IntakeSubsystem(IntakeIO io) {
    intakeIO = io;
    intakeLeftMotorDisconnectedAlert =
        new Alert("Intake Left motor disconnected!", AlertType.kError);
    intakeRightMotorDisconnectedAlert =
        new Alert("Intake Right motor disconnected!", AlertType.kError);
    extensionLeftMotorDisconnectedAlert =
        new Alert("Intake Extension Left motor disconnected!", AlertType.kError);
    extensionRightMotorDisconnectedAlert =
        new Alert("Intake Extension Right motor disconnected!", AlertType.kError);

    extensionPID.setSetpoint(IntakeConstants.extensionStartingAngleRadians);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    intakeLeftMotorDisconnectedAlert.set(!inputs.intakeLeftMotorConnected);
    intakeRightMotorDisconnectedAlert.set(!inputs.intakeRightMotorConnected);
    extensionLeftMotorDisconnectedAlert.set(!inputs.extensionLeftMotorConnected);
    extensionRightMotorDisconnectedAlert.set(!inputs.extensionRightMotorConnected);

    setExtensionVoltage(extensionPID.calculate(inputs.extensionPositionRadians));
  }

  private void setExtensionVoltage(double volts) {
    intakeIO.setExtensionVoltage(volts);
  }

  public void setIntakeVoltage(double volts) {
    intakeIO.setIntakeVoltage(volts);
  }

  public void setExtensionPosition(double positionRadians) {
    double extensionSetpoint =
        MathUtil.clamp(
            positionRadians,
            IntakeConstants.extensionMinAngleRadians,
            IntakeConstants.extensionMaxAngleRadians);
    extensionPID.setSetpoint(extensionSetpoint);
  }
}
