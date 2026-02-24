// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Sim roller implementation */
public class RollerIOSim implements RollerIO {

  private double appliedVolts;
  private DCMotorSim intakeSim;

  public RollerIOSim() {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(2), RollerConstants.momentOfInertia, RollerConstants.gearRatio),
            DCMotor.getNeo550(2));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    intakeSim.update(Constants.dtSeconds);
    inputs.positionRotations = intakeSim.getAngularPositionRotations();
    inputs.velocityRPM = intakeSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    double currentPerMotor = intakeSim.getCurrentDrawAmps() / 2.0;
    inputs.currentAmps = new double[] {currentPerMotor, currentPerMotor};
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    intakeSim.setInput(volts);
  }
}
