// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Sim flywheel implementation */
public class FlywheelIOSim implements FlywheelIO {

  private double appliedVolts;
  private FlywheelSim flywheelSim;
  private SimpleMotorFeedforward flywheelFF;
  private PIDController flywheelPID;

  public FlywheelIOSim() {
    flywheelSim =
        new FlywheelSim(
            // Hacky way to make sim "accurate enough"
            LinearSystemId.identifyVelocitySystem(
                FlywheelConstants.kv / Units.rotationsPerMinuteToRadiansPerSecond(1.0),
                FlywheelConstants.ka / (3 * Units.rotationsPerMinuteToRadiansPerSecond(1.0))),
            DCMotor.getNeoVortex(2));
    flywheelFF =
        new SimpleMotorFeedforward(
            FlywheelConstants.ks, FlywheelConstants.kv, FlywheelConstants.ka);
    flywheelPID = new PIDController(FlywheelConstants.kp * 12, 0.0, 0.0);
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

  @Override
  public void setMotorVelocity(double velocityRPM) {
    appliedVolts =
        MathUtil.clamp(
            flywheelFF.calculate(velocityRPM)
                + flywheelPID.calculate(flywheelSim.getAngularVelocityRPM(), velocityRPM),
            0.0,
            12.0);
    flywheelSim.setInput(appliedVolts);
  }
}
