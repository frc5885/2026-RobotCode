// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.util.EqualsUtil;
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
  private double extensionSetpoint = ExtensionConstants.startingPositionMeters;

  private boolean runExtensionClosedLoop =
      false; // Closed loop will enable as soon as we command a goal

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

    if (runExtensionClosedLoop) {
      extensionIO.setMotorPosition(extensionSetpoint);
    }
    visualizationUpdate();
  }

  private void setExtensionVoltage(double volts) {
    extensionIO.setMotorVoltage(volts);
  }

  public void setIntakeRollerVoltage(double volts) {
    rollerIO.setMotorVoltage(volts);
  }

  public void runExtensionOpenLoop(double volts) {
    runExtensionClosedLoop = false;
    setExtensionVoltage(volts);
  }

  public double getExtensionPosition() {
    return extensionInputs
        .leftPositionMeters; // left and right should be the same, so just return left
  }

  /** Returns true if the extension is out (more than 6 inches). */
  public boolean isExtensionOut() {
    return extensionInputs.leftPositionMeters > Units.inchesToMeters(6.0);
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

  /**
   * Commands the extension to move to the given position in meters.
   *
   * @param positionMeters The position in meters to set the extension to.
   */
  public void setExtensionPosition(double positionMeters) {
    double setPoint =
        MathUtil.clamp(
            positionMeters,
            ExtensionConstants.minExtensionMeters,
            ExtensionConstants.maxExtensionMeters);
    extensionSetpoint = setPoint;

    Logger.recordOutput("Intake/Extension/GoalPositionMeters", setPoint);
    runExtensionClosedLoop = true;
  }

  @AutoLogOutput(key = "Intake/Extension/AtSetPoint")
  public boolean isExtensionAtSetPoint() {
    return EqualsUtil.epsilonEquals(
        getExtensionPosition(), extensionSetpoint, ExtensionConstants.positionToleranceMeters);
  }

  private void visualizationUpdate() {
    // Log Pose3d
    Logger.recordOutput(
        "Mechanism3d/1-Intake",
        new Pose3d(extensionInputs.leftPositionMeters, 0.0, 0.18, Rotation3d.kZero));
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

  /**
   * Sets the brake mode of the motor.
   *
   * @param brakeModeEnabled True to enable brake mode, false to enable coast mode.
   */
  public void setBrakeMode(boolean brakeModeEnabled) {
    extensionIO.setBrakeMode(brakeModeEnabled);
    runExtensionOpenLoop(0.0);
  }

  /**
   * Returns a command that homes the intake by running the extension in reverse until it hits the
   * hard stop, then zeroing the encoders. This should only be used if the encoders lose their
   * position and need to be re-zeroed, not as part of normal operation. The command will time out
   * after 5 seconds to prevent damage to the motors in case something goes wrong with the homing
   * process.
   *
   * @return A command that homes the intake.
   */
  public Command homeIntakeCommand() {
    Debouncer extensionHomeDebouncer = new Debouncer(0.2);
    double currentThresholdAmps = 5.0; // Threshold current to detect stall, may need to be tuned
    double velocityThresholdMetersPerSecond =
        0.01; // Threshold velocity to consider the extension as stopped, may need to be tuned
    return run(() -> runExtensionOpenLoop(-2.0))
        .until(
            () ->
                extensionHomeDebouncer.calculate(
                    Math.abs(extensionInputs.leftVelocityMetersPerSecond)
                            < velocityThresholdMetersPerSecond
                        && Math.abs(extensionInputs.rightVelocityMetersPerSecond)
                            < velocityThresholdMetersPerSecond
                        && extensionInputs.leftCurrentAmps > currentThresholdAmps
                        && extensionInputs.rightCurrentAmps > currentThresholdAmps))
        .withTimeout(5.0)
        .andThen(
            () -> {
              extensionIO.resetEncoderPosition(ExtensionConstants.minExtensionMeters);
              setExtensionPosition(ExtensionConstants.minExtensionMeters);
            });
  }
}
