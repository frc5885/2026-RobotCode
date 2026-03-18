// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;

    public double positionRadians = 0.0;
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public boolean absoluteEncoder1Connected = false;
    public boolean absoluteEncoder2Connected = false;
    public double absoluteEncoder1PositionRotations = 0.0;
    public double absoluteEncoder2PositionRotations = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setMotorVoltage(double volts) {}

  /**
   * Sets the brake mode of the motor.
   *
   * @param brakeModeEnabled True to enable brake mode, false to enable coast mode.
   */
  public default void setBrakeMode(boolean brakeModeEnabled) {}
}
