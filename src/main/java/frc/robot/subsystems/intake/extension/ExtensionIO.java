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
    public double positionRadians = 0.0;
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = {0.0, 0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExtensionIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setMotorVoltage(double volts) {}
}
