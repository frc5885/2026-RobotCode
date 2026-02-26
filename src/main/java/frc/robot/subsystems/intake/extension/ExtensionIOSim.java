// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

/** Sim extension implementation */
public class ExtensionIOSim implements ExtensionIO {

  private double appliedVolts;
  private final SingleJointedArmSim extensionSim;
  private final IntakeSimulation intakeSimulation;

  public ExtensionIOSim() {
    extensionSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(2),
            ExtensionConstants.gearRatio,
            SingleJointedArmSim.estimateMOI(
                ExtensionConstants.armLengthMeters, ExtensionConstants.armMassKG),
            ExtensionConstants.armLengthMeters,
            ExtensionConstants.minAngleRadians,
            ExtensionConstants.maxAngleRadians,
            true,
            ExtensionConstants.startingAngleRadians);

    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            DriveSubsystem.getInstance().getSwerveDriveSimulation(),
            // Width of the intake
            Meters.of(DriveConstants.robotWidth),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(ExtensionConstants.intakeExtensionLengthMeters),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 30 Fuel
            HopperConstants.hopperCapacity);
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    extensionSim.update(Constants.dtSeconds);
    inputs.positionRadians = extensionSim.getAngleRads();
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

  public IntakeSimulation getIntakeSimulation() {
    return intakeSimulation;
  }

  @AutoLogOutput(key = "FieldSimulation/HopperFuelCount")
  public int getSimHopperFuelCount() {
    return intakeSimulation.getGamePiecesAmount();
  }
}
