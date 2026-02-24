// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.kicker.KickerIO;
import frc.robot.subsystems.hopper.kicker.KickerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.kicker.KickerIOSim;
import frc.robot.subsystems.hopper.kicker.KickerIOSpark;
import frc.robot.subsystems.hopper.spindexer.SpindexerIO;
import frc.robot.subsystems.hopper.spindexer.SpindexerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.spindexer.SpindexerIOSim;
import frc.robot.subsystems.hopper.spindexer.SpindexerIOSpark;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
  private static HopperSubsystem INSTANCE = null;

  public static HopperSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new HopperSubsystem(new KickerIOSpark(), new SpindexerIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new HopperSubsystem(new KickerIOSim(), new SpindexerIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new HopperSubsystem(new KickerIO() {}, new SpindexerIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert kickerDisconnectedAlert;
  private final Alert spindexerDisconnectedAlert;
  private final KickerIO kickerIO;
  private final SpindexerIO spindexerIO;
  private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();
  private final SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();

  /** Creates a new Hopper. */
  private HopperSubsystem(KickerIO kickerIO, SpindexerIO spindexerIO) {
    this.kickerIO = kickerIO;
    this.spindexerIO = spindexerIO;
    kickerDisconnectedAlert = new Alert("Kicker motor disconnected!", AlertType.kError);
    spindexerDisconnectedAlert = new Alert("Spindexer motor disconnected!", AlertType.kError);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    kickerIO.updateInputs(kickerInputs);
    spindexerIO.updateInputs(spindexerInputs);
    Logger.processInputs("Hopper/Kicker", kickerInputs);
    Logger.processInputs("Hopper/Spindexer", spindexerInputs);

    kickerDisconnectedAlert.set(!kickerInputs.motorConnected);
    spindexerDisconnectedAlert.set(!spindexerInputs.motorConnected);
  }

  public void setKickerVoltage(double volts) {
    kickerIO.setMotorVoltage(volts);
  }

  public void setSpindexerVoltage(double volts) {
    spindexerIO.setMotorVoltage(volts);
  }
}
