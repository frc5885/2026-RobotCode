package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;

/**
 * Represents a physical segment of the LED strip. Currently unused by the state-stack model but
 * preserved for future per-zone pattern overrides.
 */
public class LEDZone {
  private final String name;
  private final AddressableLEDBufferView view;

  public LEDZone(String name, AddressableLEDBufferView view) {
    this.name = name;
    this.view = view;
  }

  public String getName() {
    return name;
  }

  public AddressableLEDBufferView getView() {
    return view;
  }
}
