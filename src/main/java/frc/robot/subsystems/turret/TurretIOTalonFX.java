// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.PhoenixUtil;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  private final DutyCycleEncoder absoluteEncoder1;
  private final DutyCycleEncoder absoluteEncoder2;
  private boolean isEncoderZeroed = false;
  private final EasyCRT crt;

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

    absoluteEncoder1 = new DutyCycleEncoder(TurretConstants.absoluteEncoder1Port);
    absoluteEncoder2 = new DutyCycleEncoder(TurretConstants.absoluteEncoder2Port);

    EasyCRTConfig easyCrtConfig =
        new EasyCRTConfig(
                () -> Rotations.of(absoluteEncoder1.get()),
                () -> Rotations.of(absoluteEncoder2.get()))
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 1.0, // we don't care about this
                /* driveGearTeeth */ TurretConstants.bigGearTeeth,
                /* encoder1Pinion */ TurretConstants.absoluteEncoder1Teeth,
                /* encoder2Pinion */ TurretConstants.absoluteEncoder2Teeth)
            .withAbsoluteEncoderOffsets(
                Rotations.of(TurretConstants.absoluteEncoder1OffsetRotations),
                Rotations.of(
                    TurretConstants.absoluteEncoder2OffsetRotations)) // set after mechanical zero
            .withMechanismRange(
                Radians.of(-Math.PI),
                Radians.of(Math.PI)) // assume we aren't starting in the ~10 degree overlap
            .withMatchTolerance(Degrees.of(5.0)) // had it smaller but it was unreliable
            .withAbsoluteEncoderInversions(false, false);

    // Create the solver:
    crt = new EasyCRT(easyCrtConfig);
    Logger.recordOutput("Turret/EncoderZeroed", false);
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

    inputs.absoluteEncoder1Connected = absoluteEncoder1.isConnected();
    inputs.absoluteEncoder2Connected = absoluteEncoder2.isConnected();
    inputs.absoluteEncoder1PositionRotations = absoluteEncoder1.get();
    inputs.absoluteEncoder2PositionRotations = absoluteEncoder2.get();

    if (!isEncoderZeroed
        && inputs.motorConnected
        && inputs.absoluteEncoder1Connected
        && inputs.absoluteEncoder2Connected) {
      zeroEncoder();
    }
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setBrakeMode(boolean brakeModeEnabled) {
    NeutralModeValue mode = brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> motor.setNeutralMode(mode));
  }

  private void zeroEncoder() {
    Optional<Angle> absolutePosition = crt.getAngleOptional();
    Logger.recordOutput("Turret/CRTStatus", crt.getLastStatus().name());

    if (absolutePosition.isPresent()) {

      PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(absolutePosition.get().in(Rotations)));
      isEncoderZeroed = true;
      Logger.recordOutput("Turret/EncoderZeroed", true);
      System.out.println("Turret zeroed via CRT: " + absolutePosition.get().in(Degrees) + " deg");
    }
  }
}
