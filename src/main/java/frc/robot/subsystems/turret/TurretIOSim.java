// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Sim turret implementation */
public class TurretIOSim implements TurretIO {

  private double turretAppliedVolts;

  private DCMotorSim turretSim;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                TurretConstants.turretMomentOfInertia,
                TurretConstants.turretGearRatio),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretSim.update(0.02);
    inputs.turretPositionRadians = turretSim.getAngularPositionRad();
    inputs.turretVelocityRadiansPerSecond = turretSim.getAngularVelocityRPM();
    inputs.turretAppliedVolts = turretAppliedVolts;
    inputs.turretCurrentAmps = turretSim.getCurrentDrawAmps();
    inputs.turretMotorConnected = true;
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretAppliedVolts = volts;
    turretSim.setInput(volts);
  }
}
