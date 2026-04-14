// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.extension;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ExtensionIOSpark implements ExtensionIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;
  private final Debouncer leftMotorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rightMotorConnectedDebounce = new Debouncer(0.5);

  public ExtensionIOSpark() {
    leftMotor = new SparkMax(ExtensionConstants.leftCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(ExtensionConstants.rightCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftController = leftMotor.getClosedLoopController();
    rightController = rightMotor.getClosedLoopController();

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
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    leftMotorConfig.closedLoop.pid(
        ExtensionConstants.kp, ExtensionConstants.ki, ExtensionConstants.kd);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.apply(leftMotorConfig).inverted(ExtensionConstants.rightMotorInverted);
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    // Left motor
    sparkStickyFault = false;
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.leftPositionMeters = value);
    ifOk(
        leftMotor, leftEncoder::getVelocity, (value) -> inputs.leftVelocityMetersPerSecond = value);
    ifOk(
        leftMotor,
        new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
        (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
    ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value);
    inputs.leftMotorConnected = leftMotorConnectedDebounce.calculate(!sparkStickyFault);

    // Right motor
    sparkStickyFault = false;
    ifOk(rightMotor, rightEncoder::getPosition, (value) -> inputs.rightPositionMeters = value);
    ifOk(
        rightMotor,
        rightEncoder::getVelocity,
        (value) -> inputs.rightVelocityMetersPerSecond = value);
    ifOk(
        rightMotor,
        new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
        (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
    ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);
    inputs.rightMotorConnected = rightMotorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setMotorVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setMotorPosition(double positionMeters) {
    leftController.setSetpoint(positionMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setSetpoint(positionMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Sets encoder starting angle */
  @Override
  public boolean resetEncoderPosition(double positionMeters) {
    REVLibError leftStatus = leftEncoder.setPosition(positionMeters);
    REVLibError rightStatus = rightEncoder.setPosition(positionMeters);
    if (leftStatus != REVLibError.kOk) {
      System.out.println("Failed to zero left extension encoder: " + leftStatus);
      return false;
    }
    if (rightStatus != REVLibError.kOk) {
      System.out.println("Failed to zero right extension encoder: " + rightStatus);
      return false;
    }
    return true;
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
