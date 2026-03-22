// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE = null;
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView hopperLong;
  private final AddressableLEDBufferView turret;
  private final AddressableLEDBufferView hopperShort;
  private static LEDPattern ledState;

  public static LEDSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new LEDSubsystem();
    }
    return INSTANCE;
  }

  /** Creates a new LEDSubsystem. */
  private LEDSubsystem() {
    leds = new AddressableLED(LEDConstants.ledPort);
    buffer = new AddressableLEDBuffer(LEDConstants.length);

    leds.setLength(LEDConstants.length);
    leds.setData(buffer);
    leds.setColorOrder(ColorOrder.kRGB);

    ledState = LEDConstants.States.disabled;
    leds.start();

    hopperLong = buffer.createView(0, LEDConstants.hopperLongLength - 1);
    turret =
        buffer.createView(
            LEDConstants.hopperLongLength,
            LEDConstants.hopperLongLength + LEDConstants.turretLength - 1);
    hopperShort =
        buffer.createView(
            LEDConstants.hopperLongLength + LEDConstants.turretLength,
            LEDConstants.hopperLongLength
                + LEDConstants.turretLength
                + LEDConstants.hopperShortLength
                - 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledState.applyTo(hopperLong);
    LEDConstants.States.policeSirens.applyTo(turret);
    ledState.applyTo(hopperShort);
    leds.setData(buffer);
  }

  public static Command setLEDState(LEDPattern pattern) {
    return new InstantCommand(() -> ledState = pattern, getInstance());
  }
}
