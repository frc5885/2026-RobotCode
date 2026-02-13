// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Sim turret implementation */
public class TurretIOSim implements TurretIO {

  private double appliedVolts;

  private DCMotorSim turretSim;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), TurretConstants.momentOfInertia, TurretConstants.gearRatio),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretSim.update(Constants.dtSeconds);
    inputs.positionRadians = turretSim.getAngularPositionRad();
    inputs.velocityRadiansPerSecond = turretSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = turretAppliedVolts;
    inputs.currentAmps = turretSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    turretAppliedVolts = volts;
    turretSim.setInput(volts);
  }
}
