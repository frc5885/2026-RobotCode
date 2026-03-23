// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Sim turret implementation */
public class TurretIOSim implements TurretIO {

  private double appliedVolts;

  private DCMotorSim turretSim;
  private SimpleMotorFeedforward turretFF;
  private PIDController turretPID;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TurretConstants.kV, TurretConstants.kA),
            DCMotor.getFalcon500(1));
    turretFF =
        new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
    turretPID = new PIDController(TurretConstants.kp, TurretConstants.ki, TurretConstants.kd);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretSim.update(Constants.dtSeconds);
    inputs.positionRadians = turretSim.getAngularPositionRad();
    inputs.velocityRadiansPerSecond = turretSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = turretSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turretSim.setInput(appliedVolts);
  }

  @Override
  public void setMotorGoalPositionVelocity(
      double positionRadians, double velocityRadiansPerSecond) {
    appliedVolts =
        MathUtil.clamp(
            turretFF.calculate(velocityRadiansPerSecond)
                + turretPID.calculate(turretSim.getAngularPositionRad(), positionRadians),
            -12.0,
            12.0);
    turretSim.setInput(appliedVolts);
  }
}
