// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class HoodIOSpark implements HoodIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);
  private boolean isEncoderZeroed = false;

  private final double zeroingVoltage = 1.0;

  public HoodIOSpark() {
    motor = new SparkMax(HoodConstants.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .inverted(HoodConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HoodConstants.currentLimit)
        .voltageCompensation(12.0);
    hoodConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(HoodConstants.positionConversionFactor)
        .velocityConversionFactor(HoodConstants.velocityConversionFactor);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    hoodConfig
        .limitSwitch
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
        .forwardLimitSwitchPosition(HoodConstants.startingAngleRadians)
        .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder)
        .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRadians = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadiansPerSecond = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.motorConnected = motorConnectedDebounce.calculate(!sparkStickyFault);

    if (!isEncoderZeroed && inputs.motorConnected) {
      if (zeroEncoder() == REVLibError.kOk) {
        isEncoderZeroed = true;
        System.out.println("Hood encoder zeroed");
      }
    }
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  /** Sets encoder starting angle */
  private REVLibError zeroEncoder() {
    encoder.setPosition(HoodConstants.startingAngleRadians);
    if (!motor.getForwardLimitSwitch().isPressed()) {
      setMotorVoltage(zeroingVoltage);
    }
    return REVLibError.kOk;
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
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
