// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSpark;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.EqualsUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem INSTANCE = null;

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new ShooterSubsystem(new FlywheelIOSpark(), new HoodIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new ShooterSubsystem(new FlywheelIOSim(), new HoodIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new ShooterSubsystem(new FlywheelIO() {}, new HoodIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert flywheelLeftMotorDisconnectedAlert;
  private final Alert flywheelRightMotorDisconnectedAlert;
  private final Alert hoodMotorDisconnectedAlert;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final SysIdRoutine hoodSysId;
  private final PIDController hoodPID =
      new PIDController(HoodConstants.kp, HoodConstants.ki, HoodConstants.kd);
  private final ArmFeedforward hoodFF =
      new ArmFeedforward(HoodConstants.ks, HoodConstants.kg, HoodConstants.kv, HoodConstants.ka);
  private final BangBangController flywheelBangBangController = new BangBangController();
  private final Debouncer flywheelAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer hoodAtSetpointDebouncer = new Debouncer(0.1);

  private TrapezoidProfile hoodProfile =
      new TrapezoidProfile(
          new Constraints(
              HoodConstants.maxVelocityRadiansPerSecond,
              HoodConstants.maxAccelerationRadiansPerSecondSquared));
  private TrapezoidProfile.State hoodGoalState =
      new TrapezoidProfile.State(HoodConstants.startingAngleRadians, 0.0);
  private TrapezoidProfile.State hoodPrevSetpoint = hoodGoalState;
  private boolean runHoodClosedLoop = true;
  private boolean runFlywheelClosedLoop = false;

  /** Creates a new Shooter. */
  private ShooterSubsystem(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    flywheelLeftMotorDisconnectedAlert =
        new Alert("Shooter flywheel left motor disconnected!", AlertType.kError);
    flywheelRightMotorDisconnectedAlert =
        new Alert("Shooter flywheel right motor disconnected!", AlertType.kError);
    hoodMotorDisconnectedAlert = new Alert("Shooter hood motor disconnected!", AlertType.kError);

    hoodSysId = hoodSysIdSetup();

    hoodPID.setTolerance(HoodConstants.positionToleranceRadians);
    flywheelBangBangController.setTolerance(FlywheelConstants.velocityToleranceRPM);
    flywheelBangBangController.setSetpoint(0);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    flywheelLeftMotorDisconnectedAlert.set(!flywheelInputs.leftMotorConnected);
    flywheelRightMotorDisconnectedAlert.set(!flywheelInputs.rightMotorConnected);
    hoodMotorDisconnectedAlert.set(!hoodInputs.motorConnected);

    if (runHoodClosedLoop) {
      TrapezoidProfile.State current = getHoodCurrentState();
      TrapezoidProfile.State setpoint =
          hoodProfile.calculate(Constants.dtSeconds, hoodPrevSetpoint, hoodGoalState);
      hoodPrevSetpoint = setpoint;
      Logger.recordOutput("Shooter/Hood/SetpointPosition", setpoint.position);
      Logger.recordOutput("Shooter/Hood/SetpointVelocity", setpoint.velocity);

      setHoodVoltage(
          hoodFF.calculate(
                  setpoint.position + HoodConstants.armOffsetToHorizontalRadians, setpoint.velocity)
              + hoodPID.calculate(current.position, setpoint.position));
    }

    if (runFlywheelClosedLoop) {
      setFlywheelVoltage(flywheelBangBangController.calculate(flywheelInputs.velocityRPM) * 12.0);
    }

    visualizationUpdate();
  }

  private void setFlywheelVoltage(double volts) {
    flywheelIO.setMotorVoltage(volts);
  }

  private void setHoodVoltage(double volts) {
    hoodIO.setMotorVoltage(volts);
  }

  public void runHoodOpenLoop(double volts) {
    runHoodClosedLoop = false;
    setHoodVoltage(volts);
  }

  /**
   * Sets the hood goal state, consisting of a position and a velocity
   *
   * @param positionRadians
   * @param velocityRadiansPerSecond
   */
  public void setHoodGoal(double positionRadians, double velocityRadiansPerSecond) {
    double positionSetpoint =
        MathUtil.clamp(
            positionRadians, HoodConstants.minAngleRadians, HoodConstants.maxAngleRadians);
    double velocitySetpoint =
        MathUtil.clamp(
            velocityRadiansPerSecond,
            -HoodConstants.maxVelocityRadiansPerSecond,
            HoodConstants.maxVelocityRadiansPerSecond);
    hoodGoalState = new TrapezoidProfile.State(positionSetpoint, velocitySetpoint);
    if (!runHoodClosedLoop) {
      // Reset setpoint to current state when transitioning from open-loop
      hoodPrevSetpoint = getHoodCurrentState();
    }
    runHoodClosedLoop = true;
  }

  /**
   * Sets the hood goal to the given position with a velocity of 0
   *
   * @param positionRadians
   */
  public void setHoodGoalPosition(double positionRadians) {
    setHoodGoal(positionRadians, 0.0);
  }

  public void setFlywheelVelocity(double velocityRPM) {
    flywheelBangBangController.setSetpoint(velocityRPM);
    runFlywheelClosedLoop = true;
    // If the setpoint is 0, set the voltage to 0 because the bang bang controller will not
    // calculate a voltage in periodic if the setpoint is 0
    if (velocityRPM == 0) {
      runFlywheelClosedLoop = false;
      setFlywheelVoltage(0);
    }
    Logger.recordOutput("Shooter/FlywheelSetpoint", velocityRPM);
  }

  /** Returns debounced flywheel at setpoint */
  @AutoLogOutput(key = "Shooter/FlywheelAtSetpoint")
  public boolean isFlywheelAtSetpoint() {
    return flywheelAtSetpointDebouncer.calculate(flywheelBangBangController.atSetpoint());
  }

  /** Returns debounced hood at goal */
  @AutoLogOutput(key = "Shooter/HoodAtGoal")
  public boolean isHoodAtGoal() {
    return hoodAtSetpointDebouncer.calculate(
        EqualsUtil.epsilonEquals(
                getHoodAngle(), hoodGoalState.position, HoodConstants.positionToleranceRadians)
            && EqualsUtil.epsilonEquals(
                getHoodVelocity(),
                hoodGoalState.velocity,
                HoodConstants.velocityToleranceRadiansPerSecond));
  }

  public double getHoodAngle() {
    return hoodInputs.positionRadians;
  }

  public double getHoodVelocity() {
    return hoodInputs.velocityRadiansPerSecond;
  }

  public double getFlywheelRPM() {
    return flywheelInputs.velocityRPM;
  }

  public TrapezoidProfile.State getHoodCurrentState() {
    return new TrapezoidProfile.State(
        hoodInputs.positionRadians, hoodInputs.velocityRadiansPerSecond);
  }

  private void visualizationUpdate() {
    // Log Pose3d
    Logger.recordOutput(
        "Mechanism3d/3-Hood",
        new Pose3d(
                TurretConstants.robotToTurret.getTranslation(),
                new Rotation3d(0, 0, TurretSubsystem.getInstance().getPosition()))
            .plus(
                new Transform3d(
                    new Translation3d(0.12, 0, 0.065),
                    new Rotation3d(0, -hoodInputs.positionRadians, 0))));
  }

  // Configure SysId
  private SysIdRoutine hoodSysIdSetup() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.1).per(Second),
            Volts.of(0.2),
            null,
            (state) -> Logger.recordOutput("Shooter/Hood/SysIDState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runHoodOpenLoop(voltage.in(Volts)), null, this));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runHoodOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(hoodSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runHoodOpenLoop(0.0)).withTimeout(1.0).andThen(hoodSysId.dynamic(direction));
  }
}
