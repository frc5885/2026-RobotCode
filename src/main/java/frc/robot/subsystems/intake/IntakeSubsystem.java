// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.extension.ExtensionConstants;
import frc.robot.subsystems.intake.extension.ExtensionIO;
import frc.robot.subsystems.intake.extension.ExtensionIOInputsAutoLogged;
import frc.robot.subsystems.intake.extension.ExtensionIOSim;
import frc.robot.subsystems.intake.extension.ExtensionIOSpark;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIOSim;
import frc.robot.subsystems.intake.roller.RollerIOSpark;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem INSTANCE = null;
  private static IntakeSimulation intakeSimulation = null;

  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new IntakeSubsystem(new ExtensionIOSpark(), new RollerIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          // Maple sim setup
          intakeSimulation = setupSimIntake();
          INSTANCE = new IntakeSubsystem(new ExtensionIOSim(), new RollerIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new IntakeSubsystem(new ExtensionIO() {}, new RollerIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert extensionLeftMotorDisconnectedAlert;
  private final Alert extensionRightMotorDisconnectedAlert;
  private final Alert rollerLeftMotorDisconnectedAlert;
  private final Alert rollerRightMotorDisconnectedAlert;
  private final ExtensionIO extensionIO;
  private final RollerIO rollerIO;
  private final ExtensionIOInputsAutoLogged extensionInputs = new ExtensionIOInputsAutoLogged();
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PIDController extensionPID =
      new PIDController(ExtensionConstants.kp, ExtensionConstants.ki, ExtensionConstants.kd);

  /** Creates a new Intake. */
  private IntakeSubsystem(ExtensionIO extensionIO, RollerIO rollerIO) {
    this.extensionIO = extensionIO;
    this.rollerIO = rollerIO;
    extensionLeftMotorDisconnectedAlert =
        new Alert("Intake extension left motor disconnected!", AlertType.kError);
    extensionRightMotorDisconnectedAlert =
        new Alert("Intake extension right motor disconnected!", AlertType.kError);
    rollerLeftMotorDisconnectedAlert =
        new Alert("Intake roller left motor disconnected!", AlertType.kError);
    rollerRightMotorDisconnectedAlert =
        new Alert("Intake roller right motor disconnected!", AlertType.kError);
    extensionPID.setSetpoint(ExtensionConstants.startingAngleRadians);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    extensionIO.updateInputs(extensionInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Extension", extensionInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    extensionLeftMotorDisconnectedAlert.set(!extensionInputs.leftMotorConnected);
    extensionRightMotorDisconnectedAlert.set(!extensionInputs.rightMotorConnected);
    rollerLeftMotorDisconnectedAlert.set(!rollerInputs.leftMotorConnected);
    rollerRightMotorDisconnectedAlert.set(!rollerInputs.rightMotorConnected);

    setExtensionVoltage(extensionPID.calculate(extensionInputs.positionRadians));

    visualizationUpdate();
  }

  private void setExtensionVoltage(double volts) {
    extensionIO.setMotorVoltage(volts);
  }

  public void setIntakeRollerVoltage(double volts) {
    rollerIO.setMotorVoltage(volts);
  }

  /**
   * Returns maple sim intake simulation. MUST only be called from simulation mode or will throw an
   * error.
   */
  public IntakeSimulation getIntakeSimulation() {
    if (intakeSimulation != null) {
      return intakeSimulation;
    } else {
      throw new UnsupportedOperationException(
          "Can't call getIntakeSimulation if not in simulation mode");
    }
  }

  public void setExtensionPosition(double positionRadians) {
    double extensionSetpoint =
        MathUtil.clamp(
            positionRadians,
            ExtensionConstants.minAngleRadians,
            ExtensionConstants.maxAngleRadians);
    extensionPID.setSetpoint(extensionSetpoint);
  }

  public boolean isExtensionAtSetPoint() {
    return extensionPID.atSetpoint();
  }

  private void visualizationUpdate() {
    // Log Pose3d
    Logger.recordOutput(
        "Mechanism3d/1-Intake",
        new Pose3d(0.32, 0.0, 0.18, new Rotation3d(0.0, -extensionInputs.positionRadians, 0.0)));
  }

  private static IntakeSimulation setupSimIntake() {
    IntakeSimulation intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            DriveSubsystem.getInstance().getSwerveDriveSimulation(),
            // Width of the intake
            Meters.of(DriveConstants.robotWidth),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(ExtensionConstants.intakeExtensionLengthMeters),
            // The intake is mounted on the front side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 30 Fuel
            HopperConstants.hopperCapacity);
    intakeSim.setCustomIntakeCondition(
        (gamePiece) -> {
          // only intake if roller is spinning
          return getInstance().rollerInputs.appliedVolts > 0;
        });
    return intakeSim;
  }

  @AutoLogOutput(key = "FieldSimulation/HopperFuelCount")
  public int getSimHopperFuelCount() {
    if (intakeSimulation == null) {
      return 0;
    }
    return intakeSimulation.getGamePiecesAmount();
  }
}
