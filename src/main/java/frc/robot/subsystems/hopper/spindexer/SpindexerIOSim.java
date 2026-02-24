// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Sim spindexer implementation */
public class SpindexerIOSim implements SpindexerIO {

  private double appliedVolts;
  private DCMotorSim spindexerSim;

  public SpindexerIOSim() {
    spindexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1),
                SpindexerConstants.momentOfInertia,
                SpindexerConstants.gearRatio),
            DCMotor.getNeo550(1));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    spindexerSim.update(Constants.dtSeconds);
    inputs.positionRotations = spindexerSim.getAngularPositionRotations();
    inputs.velocityRPM = spindexerSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = spindexerSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    spindexerSim.setInput(volts);
  }
}
