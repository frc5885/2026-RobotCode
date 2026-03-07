// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Sim hood implementation */
public class HoodIOSim implements HoodIO {

  private double appliedVolts;
  private SingleJointedArmSim hoodSim;

  public HoodIOSim() {
    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            HoodConstants.gearRatio,
            SingleJointedArmSim.estimateMOI(HoodConstants.armLengthMeters, HoodConstants.armMassKG),
            HoodConstants.armLengthMeters,
            HoodConstants.minAngleRadians,
            HoodConstants.maxAngleRadians,
            true,
            HoodConstants.startingAngleRadians);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodSim.update(Constants.dtSeconds);
    inputs.positionRadians = hoodSim.getAngleRads();
    inputs.velocityRadiansPerSecond = hoodSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = hoodSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    hoodSim.setInput(volts);
  }
}
