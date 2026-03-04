// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Sim shooter implementation */
public class ShooterIOSim implements ShooterIO {

  private double flywheelAppliedVolts;
  private double hoodAppliedVolts;
  private FlywheelSim flywheelSim;
  private SingleJointedArmSim hoodSim;

  public ShooterIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(2),
                ShooterConstants.flywheelMomentOfInertia,
                ShooterConstants.flywheelGearRatio),
            DCMotor.getNeoVortex(2));

    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            ShooterConstants.hoodGearRatio,
            SingleJointedArmSim.estimateMOI(
                ShooterConstants.hoodArmLengthMeters, ShooterConstants.hoodArmMassKG),
            ShooterConstants.hoodArmLengthMeters,
            ShooterConstants.hoodMinAngleRadians,
            ShooterConstants.hoodMaxAngleRadians,
            true,
            ShooterConstants.hoodStartingAngleRadians);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(Constants.dtSeconds);
    inputs.flywheelPositionRotations = 0.0;
    inputs.flywheelVelocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelCurrentAmps =
        new double[] {flywheelSim.getCurrentDrawAmps(), flywheelSim.getCurrentDrawAmps()};
    inputs.flywheelLeftMotorConnected = true;
    inputs.flywheelRightMotorConnected = true;

    hoodSim.update(Constants.dtSeconds);
    inputs.hoodPositionRadians = hoodSim.getAngleRads();
    inputs.hoodVelocityRadiansPerSecond = hoodSim.getVelocityRadPerSec();
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();
    inputs.hoodMotorConnected = true;
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelAppliedVolts = volts;
    flywheelSim.setInput(volts);
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodAppliedVolts = volts;
    hoodSim.setInput(volts);
  }
}
