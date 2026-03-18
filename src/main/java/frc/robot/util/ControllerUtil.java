// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Utility class for controller inputs. */
public class ControllerUtil {
  public static final double movementThreshold = 0.2;

  public static boolean isLeftJoystickMoved(CommandXboxController controller) {
    return Math.abs(controller.getLeftY()) + Math.abs(controller.getLeftX()) > movementThreshold;
  }

  public static boolean isRightJoystickMoved(CommandXboxController controller) {
    return Math.abs(controller.getRightY()) + Math.abs(controller.getRightX()) > movementThreshold;
  }

  /**
   * Returns a {@link Trigger} for sprint mode, mimicking video game sprint behaviour.
   *
   * <p>Sprint activates when the left stick is pressed and remains active until the stick returns
   * to neutral AND the left stick button is released.
   *
   * @param controller the Xbox controller to read input from
   * @return a Trigger that is active while sprint mode is engaged
   */
  public static Trigger sprintToggle(CommandXboxController controller) {
    BooleanSupplier isSprinting =
        new BooleanSupplier() {
          private boolean sprinting = false;

          @Override
          public boolean getAsBoolean() {
            boolean stickPressed = controller.leftStick().getAsBoolean();
            boolean stickAtNeutral =
                Math.abs(controller.getLeftX()) < movementThreshold
                    && Math.abs(controller.getLeftY()) < movementThreshold;

            if (stickPressed) {
              sprinting = true;
            } else if (stickAtNeutral) {
              sprinting = false;
            }

            return sprinting;
          }
        };

    return new Trigger(isSprinting);
  }
}
