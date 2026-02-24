// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Sim flywheel implementation */
public class FlywheelIOSim implements FlywheelIO {

  private double appliedVolts;
  private FlywheelSim flywheelSim;

  public FlywheelIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(2),
                FlywheelConstants.momentOfInertia,
                FlywheelConstants.gearRatio),
            DCMotor.getNeoVortex(2));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelSim.update(Constants.dtSeconds);
    inputs.positionRotations = 0.0;
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    double currentPerMotor = flywheelSim.getCurrentDrawAmps() / 2.0;
    inputs.currentAmps = new double[] {currentPerMotor, currentPerMotor};
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    flywheelSim.setInput(volts);
  }
}
