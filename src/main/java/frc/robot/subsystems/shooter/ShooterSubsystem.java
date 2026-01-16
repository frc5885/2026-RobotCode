// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem INSTANCE = null;

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new ShooterSubsystem(new ShooterIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new ShooterSubsystem(new ShooterIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new ShooterSubsystem(new ShooterIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert motorDisconnectedAlert;
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Creates a new Shooter. */
  private ShooterSubsystem(ShooterIO io) {
    shooterIO = io;
    motorDisconnectedAlert = new Alert("Shooter motor disconnected!", AlertType.kError);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public void spinShooter(double volts) {
    shooterIO.setVoltage(volts);
  }
}
