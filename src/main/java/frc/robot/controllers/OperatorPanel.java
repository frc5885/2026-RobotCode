// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Interface for physical override switches on operator console. */
public class OperatorPanel {
  private final GenericHID joystick;

  public OperatorPanel(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return joystick.isConnected() && !DriverStation.getJoystickIsXbox(joystick.getPort());
  }

  /** Gets the state of a switch (0-7 from left to right). */
  public BooleanSupplier getOverrideSwitch(int index) {
    if (index < 0 || index > 7) {
      throw new RuntimeException(
          "Invalid driver override index " + Integer.toString(index) + ". Must be 0-7.");
    }
    return () -> joystick.getRawButton(index + 1);
  }

  /** Gets the negated state of a switch (0-7 from left to right). */
  public BooleanSupplier getNegatedOverrideSwitch(int index) {
    BooleanSupplier supplier = getOverrideSwitch(index);
    return () -> !supplier.getAsBoolean();
  }

  /** Returns a trigger for a switch (0-7 from left to right). */
  public Trigger overrideSwitch(int index) {
    return new Trigger(getOverrideSwitch(index));
  }
  /** Returns the coast mode switch (5th switch from the left). */
  public Trigger getCoastModeSwitch() {
    return overrideSwitch(4);
  }

  /** Returns the manual mode switch (4th switch from the left). */
  public Trigger getManualModeSwitch() {
    return overrideSwitch(3);
  }

  /** Returns the bogus call switch (8th switch from the left). */
  public Trigger getBogusCallSwitch() {
    return overrideSwitch(7);
  }
}
