// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private final SysIdRoutine extensionSysId;
  private final PIDController extensionPID =
      new PIDController(ExtensionConstants.kp, ExtensionConstants.ki, ExtensionConstants.kd);
  private final ArmFeedforward extensionFF =
      new ArmFeedforward(
          ExtensionConstants.ks,
          ExtensionConstants.kg,
          ExtensionConstants.kv,
          ExtensionConstants.ka);

  private TrapezoidProfile extensionProfile =
      new TrapezoidProfile(
          new Constraints(
              ExtensionConstants.maxVelocityRadiansPerSecond,
              ExtensionConstants.maxAccelerationRadiansPerSecondSquared));
  private TrapezoidProfile.State extensionGoalState =
      new TrapezoidProfile.State(ExtensionConstants.startingAngleRadians, 0.0);
  private TrapezoidProfile.State extensionPrevSetpoint = extensionGoalState;
  private boolean runExtensionClosedLoop = false;

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

    extensionSysId = extensionSysIdSetup();

    extensionPID.setTolerance(ExtensionConstants.positionToleranceRadians);

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
      TrapezoidProfile.State current = getExtensionCurrentState();
      TrapezoidProfile.State setpoint =
          extensionProfile.calculate(
              Constants.dtSeconds, extensionPrevSetpoint, extensionGoalState);
      extensionPrevSetpoint = setpoint;
      Logger.recordOutput("Intake/Extension/SetpointPositionRadians", setpoint.position);
      Logger.recordOutput("Intake/Extension/SetpointVelocity", setpoint.velocity);

      Logger.recordOutput(
          "Intake/Extension/FFVoltage",
          extensionFF.calculate(
              setpoint.position + ExtensionConstants.armOffsetToHorizontalRadians,
              setpoint.velocity));

      Logger.recordOutput(
          "Intake/Extension/PIDVoltage",
          extensionPID.calculate(current.position, setpoint.position));
      setExtensionVoltage(
          extensionFF.calculate(
                  setpoint.position + ExtensionConstants.armOffsetToHorizontalRadians,
                  setpoint.velocity)
              + extensionPID.calculate(current.position, setpoint.position));
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

  public TrapezoidProfile.State getExtensionCurrentState() {
    return new TrapezoidProfile.State(
        extensionInputs.positionRadians, extensionInputs.velocityRadiansPerSecond);
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
   * Commands the extension to move to the given angle in radians. This will reset the profile and
   * PID and should not be called periodically
   *
   * @param positionRadians The angle in radians to set the extension to.
   */
  public void setExtensionPosition(double positionRadians) {
    double extensionSetpoint =
        MathUtil.clamp(
            positionRadians,
            ExtensionConstants.minAngleRadians,
            ExtensionConstants.maxAngleRadians);
    extensionGoalState = new TrapezoidProfile.State(extensionSetpoint, 0.0);
    // Reset setpoint to current state
    extensionPrevSetpoint = getExtensionCurrentState();
    extensionPID.reset();
    Logger.recordOutput("Intake/Extension/GoalPositionRadians", positionRadians);
    runExtensionClosedLoop = true;
  }

  @AutoLogOutput(key = "Intake/Extension/AtSetPoint")
  public boolean isExtensionAtSetPoint() {
    return EqualsUtil.epsilonEquals(
        getExtensionCurrentState().position,
        extensionGoalState.position,
        ExtensionConstants.positionToleranceRadians);
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

  /**
   * Sets the brake mode of the motor.
   *
   * @param brakeModeEnabled True to enable brake mode, false to enable coast mode.
   */
  public void setBrakeMode(boolean brakeModeEnabled) {
    extensionIO.setBrakeMode(brakeModeEnabled);
    runExtensionOpenLoop(0.0);
  }

  // Configure SysId
  private SysIdRoutine extensionSysIdSetup() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(2.0).per(Second),
            Volts.of(2.0),
            Seconds.of(15.0),
            (state) -> Logger.recordOutput("Intake/Extension/SysIDState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runExtensionOpenLoop(voltage.in(Volts)), null, this));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runExtensionOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(extensionSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runExtensionOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(extensionSysId.dynamic(direction));
  }
}
