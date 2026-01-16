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
  private final SparkMax shooterMotor;
  private final RelativeEncoder shooterEncoder;
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public ShooterIOSpark() {
    shooterMotor = new SparkMax(ShooterConstants.canId, MotorType.kBrushless);
    shooterEncoder = shooterMotor.getEncoder();

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig
        .inverted(ShooterConstants.motorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.currentLimit)
        .voltageCompensation(12.0);
    shooterConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
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
        shooterMotor,
        5,
        () ->
            shooterMotor.configure(
                shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(shooterMotor, shooterEncoder::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(shooterMotor, shooterEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        shooterMotor,
        new DoubleSupplier[] {shooterMotor::getAppliedOutput, shooterMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(shooterMotor, shooterMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.motorConnected = motorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    shooterMotor.setVoltage(volts);
  }
}
