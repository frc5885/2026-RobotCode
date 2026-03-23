package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDZone extends SubsystemBase {
  private final AddressableLEDBufferView view;
  private LEDPattern currentPattern;

  public LEDZone(String name, AddressableLEDBufferView view, LEDPattern defaultPattern) {
    setName(name);
    this.view = view;
    this.currentPattern = defaultPattern;
    setDefaultCommand(applyPattern(defaultPattern));
  }

  @Override
  public void periodic() {
    currentPattern.applyTo(view);
  }

  /** Runs a pattern on this zone until interrupted, then reverts to default. */
  public Command applyPattern(LEDPattern pattern) {
    return startRun(() -> currentPattern = pattern, () -> {}).withName("LEDZone:" + getName());
  }
}
