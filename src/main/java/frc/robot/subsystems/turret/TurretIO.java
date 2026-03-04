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
    public boolean turretMotorConnected = false;

    public double turretPositionRadians = 0.0;
    public double turretVelocityRadiansPerSecond = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setTurretVoltage(double volts) {}
}
