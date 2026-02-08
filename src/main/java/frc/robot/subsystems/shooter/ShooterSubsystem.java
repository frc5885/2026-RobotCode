// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
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

  private final Alert flywheelLeftMotorDisconnectedAlert;
  private final Alert flywheelRightMotorDisconnectedAlert;
  private final Alert hoodMotorDisconnectedAlert;
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final PIDController hoodPID =
      new PIDController(ShooterConstants.hoodKp, ShooterConstants.hoodKi, ShooterConstants.hoodKd);
  private final BangBangController flywheelBangBangController = new BangBangController();

  /** Creates a new Shooter. */
  private ShooterSubsystem(ShooterIO io) {
    shooterIO = io;
    flywheelLeftMotorDisconnectedAlert =
        new Alert("Shooter Left Flywheel motor disconnected!", AlertType.kError);
    flywheelRightMotorDisconnectedAlert =
        new Alert("Shooter Right Flywheel motor disconnected!", AlertType.kError);
    hoodMotorDisconnectedAlert = new Alert("Shooter Hood motor disconnected!", AlertType.kError);

    hoodPID.setSetpoint(ShooterConstants.hoodStartingAngleRadians);
    flywheelBangBangController.setSetpoint(0);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    flywheelLeftMotorDisconnectedAlert.set(!inputs.flywheelLeftMotorConnected);
    flywheelRightMotorDisconnectedAlert.set(!inputs.flywheelRightMotorConnected);
    hoodMotorDisconnectedAlert.set(!inputs.hoodMotorConnected);

    setHoodVoltage(hoodPID.calculate(inputs.hoodPositionRadians));
    if (flywheelBangBangController.getSetpoint() != 0) {
      setFlywheelVoltage(flywheelBangBangController.calculate(inputs.flywheelVelocityRPM) * 12.0);
    }
  }

  private void setFlywheelVoltage(double volts) {
    shooterIO.setFlywheelVoltage(volts);
  }

  private void setHoodVoltage(double volts) {
    shooterIO.setHoodVoltage(volts);
  }

  public void setHoodPosition(double positionRadians) {
    double hoodSetpoint =
        MathUtil.clamp(
            positionRadians,
            ShooterConstants.hoodMinAngleRadians,
            ShooterConstants.hoodMaxAngleRadians);
    hoodPID.setSetpoint(hoodSetpoint);
  }

  public void setFlywheelVelocity(double velocityRPM) {
    flywheelBangBangController.setSetpoint(velocityRPM);
    // If the setpoint is 0, set the voltage to 0 because the bang bang controller will not
    // calculate a voltage in periodic if the setpoint is 0
    if (velocityRPM == 0) {
      setFlywheelVoltage(0);
    }
  }
}
