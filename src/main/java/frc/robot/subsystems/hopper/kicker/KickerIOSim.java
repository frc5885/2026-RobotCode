// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Sim kicker implementation */
public class KickerIOSim implements KickerIO {

  private double appliedVolts;
  private DCMotorSim kickerSim;

  public KickerIOSim() {
    kickerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), KickerConstants.momentOfInertia, KickerConstants.gearRatio),
            DCMotor.getNeo550(1));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    kickerSim.update(Constants.dtSeconds);
    inputs.positionRotations = kickerSim.getAngularPositionRotations();
    inputs.velocityRPM = kickerSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = kickerSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    kickerSim.setInput(volts);
  }
}
