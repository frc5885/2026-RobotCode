// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSpark;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem INSTANCE = null;

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new ShooterSubsystem(new FlywheelIOSpark(), new HoodIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new ShooterSubsystem(new FlywheelIOSim(), new HoodIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new ShooterSubsystem(new FlywheelIO() {}, new HoodIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert flywheelLeftMotorDisconnectedAlert;
  private final Alert flywheelRightMotorDisconnectedAlert;
  private final Alert hoodMotorDisconnectedAlert;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final PIDController hoodPID =
      new PIDController(HoodConstants.kp, HoodConstants.ki, HoodConstants.kd);
  private final BangBangController flywheelBangBangController = new BangBangController();

  /** Creates a new Shooter. */
  private ShooterSubsystem(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    flywheelLeftMotorDisconnectedAlert =
        new Alert("Shooter flywheel left motor disconnected!", AlertType.kError);
    flywheelRightMotorDisconnectedAlert =
        new Alert("Shooter flywheel right motor disconnected!", AlertType.kError);
    hoodMotorDisconnectedAlert = new Alert("Shooter hood motor disconnected!", AlertType.kError);

    hoodPID.setSetpoint(HoodConstants.startingAngleRadians);
    flywheelBangBangController.setSetpoint(0);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelLeftMotorDisconnectedAlert.set(!flywheelInputs.leftMotorConnected);
    flywheelRightMotorDisconnectedAlert.set(!flywheelInputs.rightMotorConnected);
    hoodMotorDisconnectedAlert.set(!hoodInputs.motorConnected);

    setHoodVoltage(hoodPID.calculate(hoodInputs.positionRadians));
    if (flywheelBangBangController.getSetpoint() != 0) {
      setFlywheelVoltage(flywheelBangBangController.calculate(flywheelInputs.velocityRPM) * 12.0);
    }

    visualizationUpdate();
  }

  private void setFlywheelVoltage(double volts) {
    flywheelIO.setMotorVoltage(volts);
  }

  private void setHoodVoltage(double volts) {
    hoodIO.setMotorVoltage(volts);
  }

  public void setHoodPosition(double positionRadians) {
    double hoodSetpoint =
        MathUtil.clamp(
            positionRadians, HoodConstants.minAngleRadians, HoodConstants.maxAngleRadians);
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

  private void visualizationUpdate() {
    // Log Pose3d
    Logger.recordOutput(
        "Mechanism3d/3-Hood",
        new Pose3d(
                TurretConstants.robotToTurret.getTranslation(),
                new Rotation3d(0, 0, TurretSubsystem.getInstance().getPosition()))
            .plus(
                new Transform3d(
                    new Translation3d(0.12, 0, 0.065),
                    new Rotation3d(0, -hoodInputs.positionRadians, 0))));
  }
}
