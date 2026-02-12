// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE = null;
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView view;
  private LEDPattern ledState;

  public static LEDSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new LEDSubsystem();
    }
    return INSTANCE;
  }

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    leds = new AddressableLED(LEDConstants.ledPort);
    buffer = new AddressableLEDBuffer(LEDConstants.length);
    view = buffer.createView(0, LEDConstants.length - 1);

    leds.setLength(LEDConstants.length);
    leds.setData(buffer);
    leds.setColorOrder(ColorOrder.kRGB);

    ledState = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25.0));
    leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledState.applyTo(view);
    leds.setData(buffer);
  }

  public void setLEDState(LEDPattern pattern) {
    ledState = pattern;
  }
}
