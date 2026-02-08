// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Utility class for controller inputs. */
public class ControllerUtil {
  public static final double movementThreshold = 0.2;

  public static boolean isLeftJoystickMoved(CommandXboxController controller) {
    return Math.abs(controller.getLeftY()) + Math.abs(controller.getLeftX()) > movementThreshold;
  }

  public static boolean isRightJoystickMoved(CommandXboxController controller) {
    return Math.abs(controller.getRightY()) + Math.abs(controller.getRightX()) > movementThreshold;
  }
}
