// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final int ledPort = 0;
  public static final int length = 79; // update this later

  public final class States {
    public static final LEDPattern disabled =
        LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25.0));
    public static final LEDPattern intakeRunning =
        LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.2));
  }
}
