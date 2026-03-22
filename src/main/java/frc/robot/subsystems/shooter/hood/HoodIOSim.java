// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Sim hood implementation */
public class HoodIOSim implements HoodIO {

  private double appliedVolts;
  private DCMotorSim hoodSim;
  private PIDController hoodPID;

  public HoodIOSim() {
    hoodSim =
        new DCMotorSim(
            LinearSystemId.identifyPositionSystem(HoodConstants.kv, HoodConstants.ka),
            DCMotor.getNeo550(1));
    hoodSim.setAngle(HoodConstants.startingAngleRadians);
    hoodPID = new PIDController(HoodConstants.kp, HoodConstants.ki, HoodConstants.kd);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodSim.update(Constants.dtSeconds);
    inputs.positionRadians = hoodSim.getAngularPositionRad();
    inputs.velocityRadiansPerSecond = hoodSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = hoodSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
    inputs.isZeroed = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    hoodSim.setInput(volts);
  }

  @Override
  public void setMotorPosition(double positionRads, double arbFFVolts) {
    appliedVolts =
        MathUtil.clamp(
            arbFFVolts + hoodPID.calculate(hoodSim.getAngularPositionRad(), positionRads),
            -12.0,
            12.0);
    hoodSim.setInput(appliedVolts);
  }
}
