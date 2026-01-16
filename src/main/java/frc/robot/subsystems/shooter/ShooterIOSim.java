// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Sim shooter implementation */
public class ShooterIOSim implements ShooterIO {

  private double appliedVolts;
  private FlywheelSim flywheelSim;

  public ShooterIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, ShooterConstants.gearRatio),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(0.02);
    inputs.positionRotations = 0.0;
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    flywheelSim.setInput(volts);
  }
}
