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
  private final SparkMax flywheelMotor;
  private final RelativeEncoder flywheelEncoder;
  private final SparkMax hoodMotor;
  private final RelativeEncoder hoodEncoder;
  private final Debouncer flywheelMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer hoodMotorConnectedDebounce = new Debouncer(0.5);

  public ShooterIOSpark() {
    flywheelMotor = new SparkMax(ShooterConstants.flywheelCanId, MotorType.kBrushless);
    flywheelEncoder = flywheelMotor.getEncoder();

    hoodMotor = new SparkMax(ShooterConstants.hoodCanId, MotorType.kBrushless);
    hoodEncoder = hoodMotor.getEncoder();

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig
        .inverted(ShooterConstants.flywheelMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.flywheelCurrentLimit)
        .voltageCompensation(12.0);
    shooterConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(ShooterConstants.flywheelPositionConversionFactor)
        .velocityConversionFactor(ShooterConstants.flywheelVelocityConversionFactor);
    shooterConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        flywheelMotor,
        5,
        () ->
            flywheelMotor.configure(
                shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

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
    sparkStickyFault = false;
    ifOk(
        flywheelMotor,
        flywheelEncoder::getPosition,
        (value) -> inputs.flywheelPositionRotations = value);
    ifOk(
        flywheelMotor, flywheelEncoder::getVelocity, (value) -> inputs.flywheelVelocityRPM = value);
    ifOk(
        flywheelMotor,
        new DoubleSupplier[] {flywheelMotor::getAppliedOutput, flywheelMotor::getBusVoltage},
        (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);
    ifOk(
        flywheelMotor,
        flywheelMotor::getOutputCurrent,
        (value) -> inputs.flywheelCurrentAmps = value);
    inputs.flywheelMotorConnected = flywheelMotorConnectedDebounce.calculate(!sparkStickyFault);

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
  public void setFlyWheelVoltage(double volts) {
    flywheelMotor.setVoltage(volts);
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodMotor.setVoltage(volts);
  }
}
