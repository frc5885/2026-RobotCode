// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean flywheelLeftMotorConnected = false;
    public boolean flywheelRightMotorConnected = false;
    public double flywheelPositionRotations = 0.0;
    public double flywheelVelocityRPM = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double[] flywheelCurrentAmps = {0.0, 0.0};

    public boolean hoodMotorConnected = false;
    public double hoodPositionRadians = 0.0;
    public double hoodVelocityRadiansPerSecond = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setFlywheelVoltage(double volts) {}

  /** Run the motor at the specified voltage. */
  public default void setHoodVoltage(double volts) {}
}
