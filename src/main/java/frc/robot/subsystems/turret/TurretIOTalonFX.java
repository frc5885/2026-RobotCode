// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  private final TalonFXConfiguration turretConfig;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(false);

  private final double timeoutSeconds = 0.25;
  private final double updateFrequency = 50.0; // Hz

  public TurretIOTalonFX() {
    motor = new TalonFX(TurretConstants.canId);

    turretConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        TurretConstants.motorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(TurretConstants.currentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withVoltage(
                new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(TurretConstants.gearRatio));

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(turretConfig, timeoutSeconds));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        updateFrequency, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    boolean motorStatus = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current);
    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus);
    inputs.positionRadians = position.getValue().in(Radians);
    inputs.velocityRadiansPerSecond = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
