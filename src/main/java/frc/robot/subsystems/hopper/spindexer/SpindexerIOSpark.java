// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.spindexer;

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

public class SpindexerIOSpark implements SpindexerIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public SpindexerIOSpark() {
    motor = new SparkMax(SpindexerConstants.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig spindexerConfig = new SparkMaxConfig();
    spindexerConfig
        .inverted(SpindexerConstants.motorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SpindexerConstants.currentLimit)
        .voltageCompensation(12.0);
    spindexerConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(SpindexerConstants.positionConversionFactor)
        .velocityConversionFactor(SpindexerConstants.velocityConversionFactor);
    spindexerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.motorConnected = motorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
