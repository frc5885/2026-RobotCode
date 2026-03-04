// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Sim intake implementation */
public class IntakeIOSim implements IntakeIO {

  private double intakeAppliedVolts;
  private double extensionAppliedVolts;
  private FlywheelSim intakeSim;
  private SingleJointedArmSim extensionSim;

  public IntakeIOSim() {
    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(2),
                IntakeConstants.intakeMomentOfInertia,
                IntakeConstants.intakeGearRatio),
            DCMotor.getNeo550(2));

    extensionSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(2),
            IntakeConstants.extensionGearRatio,
            SingleJointedArmSim.estimateMOI(
                IntakeConstants.extensionArmLengthMeters, IntakeConstants.extensionArmMassKG),
            IntakeConstants.extensionArmLengthMeters,
            IntakeConstants.extensionMinAngleRadians,
            IntakeConstants.extensionMaxAngleRadians,
            true,
            IntakeConstants.extensionStartingAngleRadians);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(Constants.dtSeconds);
    inputs.intakePositionRotations = 0.0;
    inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.intakeCurrentAmps =
        new double[] {intakeSim.getCurrentDrawAmps(), intakeSim.getCurrentDrawAmps()};
    inputs.intakeLeftMotorConnected = true;
    inputs.intakeRightMotorConnected = true;

    extensionSim.update(Constants.dtSeconds);
    inputs.extensionPositionRadians = extensionSim.getAngleRads();
    inputs.extensionVelocityRadiansPerSecond = extensionSim.getVelocityRadPerSec();
    inputs.extensionAppliedVolts = extensionAppliedVolts;
    inputs.extensionCurrentAmps =
        new double[] {extensionSim.getCurrentDrawAmps(), extensionSim.getCurrentDrawAmps()};
    inputs.extensionLeftMotorConnected = true;
    inputs.extensionRightMotorConnected = true;
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeAppliedVolts = volts;
    intakeSim.setInput(volts);
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionAppliedVolts = volts;
    extensionSim.setInput(volts);
  }
}
