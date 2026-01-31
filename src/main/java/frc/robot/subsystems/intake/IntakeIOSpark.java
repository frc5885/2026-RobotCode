// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax intakeLeftMotor;
  private final SparkMax intakeRightMotor;
  private final RelativeEncoder intakeLeftEncoder;
  private final SparkMax extensionLeftMotor;
  private final SparkMax extensionRightMotor;
  private final RelativeEncoder extensionLeftEncoder;
  private final Debouncer intakeLeftMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer intakeRightMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer extensionLeftMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer extensionRightMotorConnectedDebounce = new Debouncer(0.5);

  public IntakeIOSpark() {
    intakeLeftMotor = new SparkMax(IntakeConstants.intakeLeftCanId, MotorType.kBrushless);
    intakeRightMotor = new SparkMax(IntakeConstants.intakeRightCanId, MotorType.kBrushless);
    intakeLeftEncoder = intakeLeftMotor.getEncoder();

    extensionLeftMotor = new SparkMax(IntakeConstants.extensionLeftCanId, MotorType.kBrushless);
    extensionRightMotor = new SparkMax(IntakeConstants.extensionRightCanId, MotorType.kBrushless);
    extensionLeftEncoder = extensionLeftMotor.getEncoder();

    SparkMaxConfig intakeLeftMotorConfig = new SparkMaxConfig();
    intakeLeftMotorConfig
        .inverted(IntakeConstants.intakeLeftMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.intakeCurrentLimit)
        .voltageCompensation(12.0);
    intakeLeftMotorConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(IntakeConstants.intakePositionConversionFactor)
        .velocityConversionFactor(IntakeConstants.intakeVelocityConversionFactor);
    intakeLeftMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeLeftMotor,
        5,
        () ->
            intakeLeftMotor.configure(
                intakeLeftMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkBaseConfig intakeRightMotorConfig =
        intakeLeftMotorConfig.follow(
            intakeLeftMotor, IntakeConstants.intakeMotorsOppositeDirections);
    tryUntilOk(
        intakeRightMotor,
        5,
        () ->
            intakeRightMotor.configure(
                intakeRightMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    SparkMaxConfig extensionLeftMotorConfig = new SparkMaxConfig();
    extensionLeftMotorConfig
        .inverted(IntakeConstants.extensionLeftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.extensionCurrentLimit)
        .voltageCompensation(12.0);
    extensionLeftMotorConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(IntakeConstants.extensionPositionConversionFactor)
        .velocityConversionFactor(IntakeConstants.extensionVelocityConversionFactor);
    extensionLeftMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        extensionLeftMotor,
        5,
        () ->
            extensionLeftMotor.configure(
                extensionLeftMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkBaseConfig extensionRightMotorConfig =
        extensionLeftMotorConfig.follow(
            extensionLeftMotor, IntakeConstants.extensionMotorsOppositeDirections);
    tryUntilOk(
        extensionRightMotor,
        5,
        () ->
            intakeRightMotor.configure(
                extensionRightMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double[] intakeCurrents = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        intakeLeftMotor,
        intakeLeftEncoder::getPosition,
        (value) -> inputs.intakePositionRotations = value);
    ifOk(
        intakeLeftMotor,
        intakeLeftEncoder::getVelocity,
        (value) -> inputs.intakeVelocityRPM = value);
    ifOk(
        intakeLeftMotor,
        new DoubleSupplier[] {intakeLeftMotor::getAppliedOutput, intakeLeftMotor::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    ifOk(intakeLeftMotor, intakeLeftMotor::getOutputCurrent, (value) -> intakeCurrents[0] = value);
    inputs.intakeLeftMotorConnected = intakeLeftMotorConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        intakeRightMotor, intakeRightMotor::getOutputCurrent, (value) -> intakeCurrents[1] = value);
    inputs.intakeRightMotorConnected =
        intakeRightMotorConnectedDebounce.calculate(!sparkStickyFault);
    inputs.intakeCurrentAmps = intakeCurrents;

    double[] extensionCurrents = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        extensionLeftMotor,
        extensionLeftEncoder::getPosition,
        (value) -> inputs.extensionPositionRadians = value);
    ifOk(
        extensionLeftMotor,
        extensionLeftEncoder::getVelocity,
        (value) -> inputs.extensionVelocityRadiansPerSecond = value);
    ifOk(
        extensionLeftMotor,
        new DoubleSupplier[] {
          extensionLeftMotor::getAppliedOutput, extensionLeftMotor::getBusVoltage
        },
        (values) -> inputs.extensionAppliedVolts = values[0] * values[1]);
    ifOk(
        extensionLeftMotor,
        extensionLeftMotor::getOutputCurrent,
        (value) -> extensionCurrents[0] = value);
    inputs.extensionLeftMotorConnected =
        extensionLeftMotorConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        extensionRightMotor,
        extensionRightMotor::getOutputCurrent,
        (value) -> extensionCurrents[1] = value);
    inputs.extensionRightMotorConnected =
        extensionRightMotorConnectedDebounce.calculate(!sparkStickyFault);
    inputs.extensionCurrentAmps = extensionCurrents;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setIntakeVoltage(double volts) {
    intakeLeftMotor.setVoltage(volts);
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionLeftMotor.setVoltage(volts);
  }
}
