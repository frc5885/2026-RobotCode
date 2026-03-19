// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FlywheelIOSpark implements FlywheelIO {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer leftMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rightMotorConnectedDebounce = new Debouncer(0.5);

  public FlywheelIOSpark() {
    leftMotor = new SparkFlex(FlywheelConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new SparkFlex(FlywheelConstants.rightCanId, MotorType.kBrushless);
    encoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();

    SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig
        .inverted(FlywheelConstants.leftMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FlywheelConstants.currentLimit)
        .voltageCompensation(12.0);
    leftMotorConfig
        .encoder
        .uvwMeasurementPeriod(8)
        .uvwAverageDepth(2)
        .quadratureMeasurementPeriod(8)
        .positionConversionFactor(FlywheelConstants.positionConversionFactor)
        .velocityConversionFactor(FlywheelConstants.velocityConversionFactor);
    leftMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    leftMotorConfig.closedLoop.feedForward.sva(
        FlywheelConstants.ks, FlywheelConstants.kv, FlywheelConstants.ka);
    leftMotorConfig.closedLoop.pid(
        FlywheelConstants.kp, FlywheelConstants.ki, FlywheelConstants.kd);
    leftMotorConfig.closedLoop.minOutput(0.0, ClosedLoopSlot.kSlot0); // don't allow negative
    leftMotorConfig.closedLoop.maxOutput(1.0, ClosedLoopSlot.kSlot0);
    leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // output
    // .allowedClosedLoopError(FlywheelConstants.velocityToleranceRPM, ClosedLoopSlot.kSlot0);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkBaseConfig rightMotorConfig =
        leftMotorConfig.follow(leftMotor, FlywheelConstants.motorsOppositeDirections);
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double[] currents = {0.0, 0.0};

    // Left motor
    sparkStickyFault = false;
    ifOk(leftMotor, encoder::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(leftMotor, encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        leftMotor,
        new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> currents[0] = value);
    inputs.leftMotorConnected = leftMotorConnectedDebounce.calculate(!sparkStickyFault);
    Logger.recordOutput("Shooter/Flywheel/AppliedOutput", leftMotor.getAppliedOutput());

    // Right motor
    sparkStickyFault = false;
    ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> currents[1] = value);
    inputs.rightMotorConnected = rightMotorConnectedDebounce.calculate(!sparkStickyFault);

    inputs.currentAmps = currents;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setMotorVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setMotorVelocity(double velocityRPM) {
    controller.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
