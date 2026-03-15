// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class ExtensionIOSpark implements ExtensionIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder encoder;
  private final SparkAbsoluteEncoder absoluteEncoder;
  private final Debouncer leftMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rightMotorConnectedDebounce = new Debouncer(0.5);
  private boolean isEncoderZeroed = false;

  public ExtensionIOSpark() {
    leftMotor = new SparkMax(ExtensionConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(ExtensionConstants.rightCanId, MotorType.kBrushless);
    encoder = leftMotor.getEncoder();
    absoluteEncoder = leftMotor.getAbsoluteEncoder();

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig
        .inverted(ExtensionConstants.leftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ExtensionConstants.currentLimit)
        .voltageCompensation(12.0);
    leftMotorConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(ExtensionConstants.positionConversionFactor)
        .velocityConversionFactor(ExtensionConstants.velocityConversionFactor);
    leftMotorConfig
        .absoluteEncoder
        .positionConversionFactor(Units.rotationsToRadians(1.0))
        .velocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1.0));
    leftMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkBaseConfig rightMotorConfig =
        leftMotorConfig.follow(leftMotor, ExtensionConstants.motorsOppositeDirections);
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    double[] currents = {0.0, 0.0};

    // Left motor
    sparkStickyFault = false;
    ifOk(leftMotor, encoder::getPosition, (value) -> inputs.positionRadians = value);
    ifOk(
        leftMotor,
        absoluteEncoder::getPosition,
        (value) ->
            inputs.absolutePositionRadians = value - ExtensionConstants.absoluteEncoderOffset);
    ifOk(leftMotor, encoder::getVelocity, (value) -> inputs.velocityRadiansPerSecond = value);
    ifOk(
        leftMotor,
        new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> currents[0] = value);
    inputs.leftMotorConnected = leftMotorConnectedDebounce.calculate(!sparkStickyFault);

    // Right motor
    sparkStickyFault = false;
    ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> currents[1] = value);
    inputs.rightMotorConnected = rightMotorConnectedDebounce.calculate(!sparkStickyFault);

    inputs.currentAmps = currents;

    if (!isEncoderZeroed && inputs.leftMotorConnected) {
      if (zeroEncoder(inputs.absolutePositionRadians) == REVLibError.kOk) {
        isEncoderZeroed = true;
        System.out.println("Intake extension encoder zeroed");
      }
    }
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setMotorVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  /** Sets encoder starting angle */
  private REVLibError zeroEncoder(double absolutePosition) {
    return encoder.setPosition(absolutePosition);
  }

  /**
   * Sets the brake mode of the motor.
   *
   * @param brakeModeEnabled True to enable brake mode, false to enable coast mode.
   */
  @Override
  public void setBrakeMode(boolean brakeModeEnabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
