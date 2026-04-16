// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  @AutoLog
  public static class ExtensionIOInputs {
    public boolean leftMotorConnected = false;
    public boolean rightMotorConnected = false;
    public double leftPositionMeters = 0.0;
    public double rightPositionMeters = 0.0;
    public double leftVelocityMetersPerSecond = 0.0;
    public double rightVelocityMetersPerSecond = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExtensionIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setMotorVoltage(double volts) {}

  /** Run the motor to position. */
  public default void setMotorPosition(double positionMeters) {}

  /** Run the left and right motors to independent positions (for sync correction). */
  public default void setMotorPositions(double leftPositionMeters, double rightPositionMeters) {
    setMotorPosition(leftPositionMeters);
  }

  /** Sets the encoder position of the motor (for homing). */
  public default boolean resetEncoderPosition(double positionMeters) {
    return false;
  }

  /**
   * Sets the brake mode of the motor.
   *
   * @param brakeModeEnabled True to enable brake mode, false to enable coast mode.
   */
  public default void setBrakeMode(boolean brakeModeEnabled) {}
}
