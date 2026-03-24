package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.leds.LEDConstants.LEDState;
import java.util.Comparator;
import java.util.TreeSet;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE = null;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  // Zones kept for future per-zone use
  private final LEDZone hopperLong;
  private final LEDZone turret;
  private final LEDZone hopperShort;

  private final TreeSet<LEDState> activeStates =
      new TreeSet<>(
          Comparator.comparingInt((LEDState s) -> s.priority).thenComparingInt(Enum::ordinal));

  public static LEDSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new LEDSubsystem();
    return INSTANCE;
  }

  private LEDSubsystem() {
    leds = new AddressableLED(LEDConstants.ledPort);
    buffer = new AddressableLEDBuffer(LEDConstants.length);

    leds.setLength(LEDConstants.length);
    leds.setColorOrder(ColorOrder.kRGB);
    leds.start();

    hopperLong = new LEDZone("HopperLong", buffer.createView(0, LEDConstants.hopperLongLength - 1));

    turret =
        new LEDZone(
            "Turret",
            buffer.createView(
                LEDConstants.hopperLongLength,
                LEDConstants.hopperLongLength + LEDConstants.turretLength - 1));

    hopperShort =
        new LEDZone(
            "HopperShort",
            buffer.createView(
                LEDConstants.hopperLongLength + LEDConstants.turretLength,
                LEDConstants.hopperLongLength
                    + LEDConstants.turretLength
                    + LEDConstants.hopperShortLength
                    - 1));

    // DISABLED is always in the stack
    activeStates.add(LEDState.DISABLED);

    // Idle while enabled
    new Trigger(DriverStation::isEnabled).whileTrue(applyState(LEDState.IDLE));
  }

  @Override
  public void periodic() {
    composePattern().applyTo(buffer);
    leds.setData(buffer);
  }

  private LEDPattern composePattern() {
    LEDPattern composite = null;
    for (LEDState state : activeStates) {
      if (composite == null) {
        composite = state.pattern;
      } else if (state.isOverlay) {
        composite = state.pattern.overlayOn(composite);
      } else {
        composite = state.pattern;
      }
    }
    return composite != null ? composite : LEDConstants.LEDState.DISABLED.pattern;
  }

  /**
   * Returns a command that adds the state to the stack on start and removes it on end. This command
   * requires NO subsystem, so multiple can run simultaneously.
   */
  public Command applyState(LEDState state) {
    return Commands.startEnd(() -> activeStates.add(state), () -> activeStates.remove(state))
        .withName("LEDState:" + state.name())
        .ignoringDisable(true);
  }

  /**
   * Adds a state to the stack. Must explicitly remove the state when done. It is preferred to use
   * applyState (Command) instead.
   *
   * @param state The state to add.
   */
  public void addState(LEDState state) {
    activeStates.add(state);
  }

  /**
   * Removes a state from the stack.
   *
   * @param state The state to remove.
   */
  public void removeState(LEDState state) {
    activeStates.remove(state);
  }

  public LEDZone getHopperLong() {
    return hopperLong;
  }

  public LEDZone getTurret() {
    return turret;
  }

  public LEDZone getHopperShort() {
    return hopperShort;
  }
}
