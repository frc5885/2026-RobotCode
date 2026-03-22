package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE = null;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  public final LEDZone hopperLong;
  public final LEDZone turret;
  public final LEDZone hopperShort;

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

    hopperLong =
        new LEDZone(
            "HopperLong",
            buffer.createView(0, LEDConstants.hopperLongLength - 1),
            LEDConstants.States.disabled);

    turret =
        new LEDZone(
            "Turret",
            buffer.createView(
                LEDConstants.hopperLongLength,
                LEDConstants.hopperLongLength + LEDConstants.turretLength - 1),
            LEDConstants.States.disabled);

    hopperShort =
        new LEDZone(
            "HopperShort",
            buffer.createView(
                LEDConstants.hopperLongLength + LEDConstants.turretLength,
                LEDConstants.hopperLongLength
                    + LEDConstants.turretLength
                    + LEDConstants.hopperShortLength
                    - 1),
            LEDConstants.States.disabled);
  }

  @Override
  public void periodic() {
    // Zones write to buffer views in their own periodics;
    // this flushes the final buffer state to hardware each tick.
    leds.setData(buffer);
  }

  /**
   * Applies a pattern across the entire physical strip as one continuous surface. Requires all
   * three zones -- interrupts their individual commands while active. When this command ends, each
   * zone reverts to its own default command.
   */
  public Command fullStripPattern(LEDPattern pattern) {
    return Commands.run(() -> pattern.applyTo(buffer), hopperLong, turret, hopperShort)
        .withName("FullStrip");
  }
}
