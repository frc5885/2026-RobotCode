// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE = null;
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView view;
  private static LEDPattern ledState;

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

    // ledState = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25.0));
    ledState = LEDPattern.solid(Color.kRed);
    // ledState =
    //     LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kBlack)
    //         .scrollAtRelativeSpeed(Percent.per(Second).of(100.0));
    // ledState =
    //     LEDPattern.solid(Color.kRed)
    //         .blink(Seconds.of(0.1))
    //         .overlayOn(LEDPattern.solid(Color.kBlue));
    ledState = LEDConstants.States.disabled;
    leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledState.applyTo(view);
    leds.setData(buffer);
  }

  public static Command setLEDState(LEDPattern pattern) {
    return new InstantCommand(() -> ledState = pattern);
  }
}
