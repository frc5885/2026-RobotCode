// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ShooterIOSpark implements ShooterIO {
  private final SparkMax flywheelLeftMotor;
  private final SparkMax flywheelRightMotor;
  private final RelativeEncoder flywheelLeftEncoder;
  private final SparkMax hoodMotor;
  private final RelativeEncoder hoodEncoder;
  private final Debouncer flywheelMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer hoodMotorConnectedDebounce = new Debouncer(0.5);

  public ShooterIOSpark() {
    flywheelLeftMotor = new SparkMax(ShooterConstants.flywheelLeftCanId, MotorType.kBrushless);
    flywheelRightMotor = new SparkMax(ShooterConstants.flywheelRightCanId, MotorType.kBrushless);
    flywheelLeftEncoder = flywheelLeftMotor.getEncoder();

    hoodMotor = new SparkMax(ShooterConstants.hoodCanId, MotorType.kBrushless);
    hoodEncoder = hoodMotor.getEncoder();

    SparkMaxConfig flywheelLeftMotorConfig = new SparkMaxConfig();
    flywheelLeftMotorConfig
        .inverted(ShooterConstants.flywheelLeftMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.flywheelCurrentLimit)
        .voltageCompensation(12.0);
    flywheelLeftMotorConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(ShooterConstants.flywheelPositionConversionFactor)
        .velocityConversionFactor(ShooterConstants.flywheelVelocityConversionFactor);
    flywheelLeftMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelLeftMotor,
        5,
        () ->
            flywheelLeftMotor.configure(
                flywheelLeftMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkMaxConfig flywheelRightMotorConfig = flywheelLeftMotorConfig;
    flywheelRightMotorConfig.follow(
        flywheelLeftMotor, ShooterConstants.flywheelMotorsOppositeDirections);
    tryUntilOk(
        flywheelRightMotor,
        5,
        () ->
            flywheelRightMotor.configure(
                flywheelRightMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .inverted(ShooterConstants.hoodMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.hoodCurrentLimit)
        .voltageCompensation(12.0);
    hoodConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(ShooterConstants.hoodPositionConversionFactor)
        .velocityConversionFactor(ShooterConstants.hoodVelocityConversionFactor);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        hoodMotor,
        5,
        () ->
            hoodMotor.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    double[] flywheelCurrents = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        flywheelLeftMotor,
        flywheelLeftEncoder::getPosition,
        (value) -> inputs.flywheelPositionRotations = value);
    ifOk(
        flywheelLeftMotor,
        flywheelLeftEncoder::getVelocity,
        (value) -> inputs.flywheelVelocityRPM = value);
    ifOk(
        flywheelLeftMotor,
        new DoubleSupplier[] {
          flywheelLeftMotor::getAppliedOutput, flywheelLeftMotor::getBusVoltage
        },
        (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);
    ifOk(
        flywheelLeftMotor,
        flywheelLeftMotor::getOutputCurrent,
        (value) -> flywheelCurrents[0] = value);
    inputs.flywheelLeftMotorConnected = flywheelMotorConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        flywheelRightMotor,
        flywheelRightMotor::getOutputCurrent,
        (value) -> flywheelCurrents[1] = value);
    inputs.flywheelRightMotorConnected =
        flywheelMotorConnectedDebounce.calculate(!sparkStickyFault);
    inputs.flywheelCurrentAmps = flywheelCurrents;

    sparkStickyFault = false;
    ifOk(hoodMotor, hoodEncoder::getPosition, (value) -> inputs.hoodPositionRadians = value);
    ifOk(
        hoodMotor,
        hoodEncoder::getVelocity,
        (value) -> inputs.hoodVelocityRadiansPerSecond = value);
    ifOk(
        hoodMotor,
        new DoubleSupplier[] {hoodMotor::getAppliedOutput, hoodMotor::getBusVoltage},
        (values) -> inputs.hoodAppliedVolts = values[0] * values[1]);
    ifOk(hoodMotor, hoodMotor::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
    inputs.hoodMotorConnected = hoodMotorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelLeftMotor.setVoltage(volts);
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodMotor.setVoltage(volts);
  }
}
