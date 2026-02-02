// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeLeftMotorConnected = false;
    public boolean intakeRightMotorConnected = false;
    public double intakePositionRotations = 0.0;
    public double intakeVelocityRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = {0.0, 0.0};

    public boolean extensionLeftMotorConnected = false;
    public boolean extensionRightMotorConnected = false;
    public double extensionPositionRadians = 0.0;
    public double extensionVelocityRadiansPerSecond = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double[] extensionCurrentAmps = {0.0, 0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setIntakeVoltage(double volts) {}

  /** Run the motor at the specified voltage. */
  public default void setExtensionVoltage(double volts) {}
}
