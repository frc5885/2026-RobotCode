// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Sim extension implementation */
public class ExtensionIOSim implements ExtensionIO {

  private double appliedVolts;
  private final ElevatorSim extensionSim;

  private final PIDController extensionPID =
      new PIDController(ExtensionConstants.kp, ExtensionConstants.ki, ExtensionConstants.kd);

  public ExtensionIOSim() {
    extensionSim =
        new ElevatorSim(
            DCMotor.getNEO(2),
            ExtensionConstants.gearRatio,
            1.0,
            ExtensionConstants.driveGearRadiusMeters,
            ExtensionConstants.minExtensionMeters,
            ExtensionConstants.maxExtensionMeters,
            false,
            ExtensionConstants.startingPositionMeters);
    extensionPID.setTolerance(ExtensionConstants.positionToleranceMeters);
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    extensionSim.update(Constants.dtSeconds);
    inputs.leftPositionMeters = extensionSim.getPositionMeters();
    inputs.rightPositionMeters = extensionSim.getPositionMeters();
    inputs.leftVelocityMetersPerSecond = extensionSim.getVelocityMetersPerSecond();
    inputs.rightVelocityMetersPerSecond = extensionSim.getVelocityMetersPerSecond();
    inputs.leftAppliedVolts = appliedVolts;
    inputs.rightAppliedVolts = appliedVolts;
    double currentPerMotor = extensionSim.getCurrentDrawAmps() / 2.0;
    inputs.leftCurrentAmps = currentPerMotor;
    inputs.rightCurrentAmps = currentPerMotor;
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
    extensionSim.setInput(volts);
  }

  @Override
  public void setMotorPosition(double positionMeters) {
    appliedVolts =
        MathUtil.clamp(
            extensionPID.calculate(extensionSim.getPositionMeters(), positionMeters), -12.0, 12.0);
    extensionSim.setInput(appliedVolts);
  }
}
