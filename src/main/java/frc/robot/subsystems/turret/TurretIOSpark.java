// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

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

public class TurretIOSpark implements TurretIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public TurretIOSpark() {

    motor = new SparkMax(TurretConstants.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig
        .inverted(TurretConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.currentLimit)
        .voltageCompensation(12.0);
    turretConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(TurretConstants.positionConversionFactor)
        .velocityConversionFactor(TurretConstants.velocityConversionFactor);
    turretConfig
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
                turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRadians = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadiansPerSecond = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.motorConnected = motorConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean brakeModeEnabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
