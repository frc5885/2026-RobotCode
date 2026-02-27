// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem INSTANCE = null;

  public static TurretSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new TurretSubsystem(new TurretIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new TurretSubsystem(new TurretIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new TurretSubsystem(new TurretIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert turretMotorDisconnectedAlert =
      new Alert("Turret motor disconnected!", AlertType.kWarning);
  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private final PIDController pidController =
      new PIDController(TurretConstants.kp, TurretConstants.ki, TurretConstants.kd);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
  private boolean runClosedLoop = false;
  private final Debouncer atGoalDebouncer = new Debouncer(0.1);

  // Stuff for tracking
  @AutoLogOutput(key = "Turret/LaunchState")
  private LaunchState launchState = LaunchState.TRACKING;

  @AutoLogOutput(key = "Turret/TargetType")
  private TargetType targetType = TargetType.FIELD_RELATIVE;

  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;
  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              TurretConstants.maxVelocityRadiansPerSecond,
              TurretConstants.maxAccelerationRadiansPerSecondSquared));
  private State setpoint = new State();

  @AutoLogOutput(key = "Turret/AtGoal")
  private boolean atGoal = false;

  /** Creates a new Turret. */
  private TurretSubsystem(TurretIO io) {
    turretIO = io;
    sysId = sysIdSetup();

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretIO.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    turretMotorDisconnectedAlert.set(!inputs.motorConnected);

    // Stuff for tracking
    if (runClosedLoop) {
      Rotation2d robotAngle = DriveSubsystem.getInstance().getRotation();
      double robotAngularVelocity =
          DriveSubsystem.getInstance().getFieldRelativeChassisSpeeds().omegaRadiansPerSecond;

      State robotRelativeGoalState =
          switch (targetType) {
            case FIELD_RELATIVE -> new State(
                goalAngle.minus(robotAngle).getRadians(), goalVelocity - robotAngularVelocity);
            case ROBOT_RELATIVE -> new State(goalAngle.getRadians(), goalVelocity);
          };
      Rotation2d robotRelativeGoalAngle = new Rotation2d(robotRelativeGoalState.position);
      double robotRelativeGoalVelocity = robotRelativeGoalState.velocity;

      boolean hasBestAngle = false;
      double bestAngle = 0;
      double minLegalAngle =
          switch (launchState) {
            case ACTIVE_LAUNCHING -> TurretConstants.minAngle;
            case TRACKING -> TurretConstants.trackMinAngle;
          };
      double maxLegalAngle =
          switch (launchState) {
            case ACTIVE_LAUNCHING -> TurretConstants.maxAngle;
            case TRACKING -> TurretConstants.trackMaxAngle;
          };

      for (int i = -2; i < 3; i++) {
        double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
        if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }
      lastGoalAngle = bestAngle;

      State goalState =
          new State(
              MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), robotRelativeGoalVelocity);

      setpoint = profile.calculate(Constants.dtSeconds, setpoint, goalState);
      atGoal =
          EqualsUtil.epsilonEquals(
                  getPosition(), bestAngle, TurretConstants.turretPositionToleranceRadians)
              && EqualsUtil.epsilonEquals(
                  getVelocity(),
                  robotRelativeGoalVelocity,
                  TurretConstants.turretVelocityToleranceRadiansPerSecond);

      Logger.recordOutput("Turret/GoalPositionRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
      Logger.recordOutput("Turret/SetpointPositionRad", setpoint.position);
      Logger.recordOutput("Turret/SetpointVelocityRadPerSec", setpoint.velocity);

      switch (targetType) {
        case FIELD_RELATIVE -> {
          turretIO.setMotorVoltage(
              pidController.calculate(
                      inputs.positionRadians, setpoint.position - TurretConstants.turretOffset)
                  + feedforward.calculate(setpoint.velocity));
        }
        case ROBOT_RELATIVE -> {
          turretIO.setMotorVoltage(
              pidController.calculate(
                  inputs.positionRadians, goalAngle.getRadians() - TurretConstants.turretOffset));
        }
      }
    }

    visualizationUpdate();
  }

  private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
    this.targetType = TargetType.FIELD_RELATIVE;
    this.goalAngle = angle;
    this.goalVelocity = velocity;
  }

  private void setRobotRelativeTarget(Rotation2d angle) {
    this.targetType = TargetType.ROBOT_RELATIVE;
    this.goalAngle = angle;
    this.goalVelocity = 0.0;
  }

  @AutoLogOutput(key = "Turret/MeasuredPositionRad")
  public double getPosition() {
    return inputs.positionRadians + TurretConstants.turretOffset;
  }

  @AutoLogOutput(key = "Turret/MeasuredVelocityRadPerSec")
  public double getVelocity() {
    return inputs.velocityRadiansPerSecond;
  }

  /** Run the turret in tracking mode. Tracks the result of the launch calculator */
  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = LaunchCalculator.getInstance().getParameters();
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          runClosedLoop = true;
        });
  }

  /** Lock to a supplied field-relative angle and velocity */
  public Command runFieldRelativeFixedCommand(Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return run(
        () -> {
          setFieldRelativeTarget(angle.get(), velocity.getAsDouble());
          runClosedLoop = true;
        });
  }

  /**
   * Lock to a supplied robot-relative angle. Only should be used for testing or if vision is not
   * available
   */
  public Command runRobotRelativeFixedCommand(Supplier<Rotation2d> angle) {
    return run(
        () -> {
          setRobotRelativeTarget(angle.get());
          runClosedLoop = true;
        });
  }

  /**
   * Set the turret to active launching mode for the duration of the command, reverting back to
   * tracking mode when done
   */
  public Command setActiveLaunchingModeCommand() {
    return new StartEndCommand(
        () -> launchState = LaunchState.ACTIVE_LAUNCHING, () -> launchState = LaunchState.TRACKING);
  }

  /** Run the turret in open loop mode at a specified voltage */
  public void runOpenLoop(double volts) {
    runClosedLoop = false;
    turretIO.setMotorVoltage(volts);
  }

  public enum LaunchState {
    ACTIVE_LAUNCHING,
    TRACKING
  }

  public enum TargetType {
    FIELD_RELATIVE,
    ROBOT_RELATIVE
  }

  /** Returns debounced atGoal (position and velocity setpoints) */
  public boolean isAtGoal() {
    if (!runClosedLoop) {
      return false;
    }
    return atGoalDebouncer.calculate(atGoal);
  }

  // Configure SysId
  private SysIdRoutine sysIdSetup() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second),
            Volts.of(3.0),
            null,
            (state) -> Logger.recordOutput("Turret/SysIDState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runOpenLoop(voltage.in(Volts)), null, this));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runOpenLoop(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runOpenLoop(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  private void visualizationUpdate() {
    // Log Pose3d
    Logger.recordOutput(
        "Mechanism3d/2-Turret",
        new Pose3d(-0.16, 0.16, 0.38, new Rotation3d(0.0, 0.0, getPosition())));
  }
}
