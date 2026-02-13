// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem INSTANCE = null;

  public static TurretSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new TurretSubsystem(new TurretIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new TurretSubsystem(new TurretIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new TurretSubsystem(new TurretIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert turretMotorDisconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kWarning);
  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final PIDController pidController =
      new PIDController(
          TurretConstants.kp, TurretConstants.ki, TurretConstants.kd);
  private boolean runClosedLoop = false;

  /** Creates a new Turret. */
  private TurretSubsystem(TurretIO io) {
    turretIO = io;

    pidController.setSetpoint(TurretConstants.startingAngleRadians);
    runClosedLoop = true;

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretIO.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    turretMotorDisconnectedAlert.set(!inputs.motorConnected);

    if (runClosedLoop) {
      turretIO.setVoltage(pidController.calculate(inputs.positionRadians));
    }
  }

  public void runOpenLoop(double volts) {
    runClosedLoop = false;
    turretIO.setVoltage(volts);
  }

  public void runClosedLoop(double positionRadians) {
    runClosedLoop = true;
    pidController.setSetpoint(positionRadians);
  }
}
