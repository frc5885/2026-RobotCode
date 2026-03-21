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
  // 42 leds for hopper
  // leds for turret
  // total
  public static final int length = 79;

  public static final class States {
    public static final LEDPattern disabled =
        LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25.0));
    public static final LEDPattern intakeRunning =
        LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.2));
    public static final LEDPattern cyanScrollingGradient =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kBlack)
            .scrollAtRelativeSpeed(Percent.per(Second).of(100.0));
    public static final LEDPattern policeSirens =
        LEDPattern.solid(Color.kRed)
            .blink(Seconds.of(0.1))
            .overlayOn(LEDPattern.solid(Color.kBlue));
  }
}
