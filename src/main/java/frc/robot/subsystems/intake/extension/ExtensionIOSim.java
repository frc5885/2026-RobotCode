// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Sim extension implementation */
public class ExtensionIOSim implements ExtensionIO {

  private double appliedVolts;
  private final SingleJointedArmSim extensionSim;

  public ExtensionIOSim() {
    extensionSim =
        new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(ExtensionConstants.kv, ExtensionConstants.ka),
            DCMotor.getNEO(2),
            ExtensionConstants.gearRatio,
            ExtensionConstants.armLengthMeters,
            ExtensionConstants.minAngleRadians,
            ExtensionConstants.maxAngleRadians,
            false,
            ExtensionConstants.startingAngleRadians);
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    extensionSim.update(Constants.dtSeconds);
    inputs.positionRadians = extensionSim.getAngleRads();
    inputs.absolutePositionRadians = extensionSim.getAngleRads();
    inputs.velocityRadiansPerSecond = extensionSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    double currentPerMotor = extensionSim.getCurrentDrawAmps() / 2.0;
    inputs.currentAmps = new double[] {currentPerMotor, currentPerMotor};
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    extensionSim.setInput(volts);
  }
}
