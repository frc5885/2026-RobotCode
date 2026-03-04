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

  private final SparkMax turretMotor;
  private final RelativeEncoder turretEncoder;

  private final Debouncer turretMotorConnectedDebounce = new Debouncer(0.5);

  public TurretIOSpark() {

    turretMotor = new SparkMax(TurretConstants.turretCanId, MotorType.kBrushless);
    turretEncoder = turretMotor.getEncoder();

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig
        .inverted(TurretConstants.turretMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.turretCurrentLimit)
        .voltageCompensation(12.0);
    turretConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(TurretConstants.turretPositionConversionFactor)
        .velocityConversionFactor(TurretConstants.turretVelocityConversionFactor);
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
        turretMotor,
        5,
        () ->
            turretMotor.configure(
                turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    sparkStickyFault = false;
    ifOk(turretMotor, turretEncoder::getPosition, (value) -> inputs.turretPositionRadians = value);
    ifOk(
        turretMotor,
        turretEncoder::getVelocity,
        (value) -> inputs.turretVelocityRadiansPerSecond = value);
    ifOk(
        turretMotor,
        new DoubleSupplier[] {turretMotor::getAppliedOutput, turretMotor::getBusVoltage},
        (values) -> inputs.turretAppliedVolts = values[0] * values[1]);
    ifOk(turretMotor, turretMotor::getOutputCurrent, (value) -> inputs.turretCurrentAmps = value);
    inputs.turretMotorConnected = turretMotorConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }
}
