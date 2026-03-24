// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public class LEDConstants {
  public static final int ledPort = 4;
  // 41 leds for hopper (starting by climber)
  // 33 leds for turret
  // 11 leds for rest of hopper
  // 85 total
  public static final int length = 85;
  public static final int hopperLongLength = 41;
  public static final int turretLength = 33;
  public static final int hopperShortLength = 11;

  public static final class States {
    private static final Distance ledSpacing = Meters.of(1.0 / 60);
    private static final LinearVelocity scrollSpeed = InchesPerSecond.of(6.0);

    public static final LEDPattern disabled =
        LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(scrollSpeed, ledSpacing);
    public static final LEDPattern intakeRunning =
        LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.2));
    public static final LEDPattern cyanScrollingGradient =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kBlack)
            .scrollAtRelativeSpeed(Percent.per(Second).of(100.0));
    public static final LEDPattern eric =
        LEDPattern.steps(Map.of(0.95, Color.kLightBlue))
            .scrollAtRelativeSpeed(Percent.per(Second).of(200.0))
            .overlayOn(
                LEDPattern.steps(Map.of(0.95, Color.kLightBlue))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(200.0))
                    .reversed())
            .overlayOn(
                LEDPattern.steps(Map.of(0.80, Color.kCadetBlue))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(150.0)))
            .overlayOn(
                LEDPattern.steps(Map.of(0.80, Color.kCadetBlue))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(150.0))
                    .reversed())
            .overlayOn(
                LEDPattern.steps(Map.of(0.90, Color.kAntiqueWhite))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(175.0)))
            .overlayOn(
                LEDPattern.steps(Map.of(0.90, Color.kAntiqueWhite))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(175.0))
                    .reversed());
    public static final LEDPattern policeSirens =
        LEDPattern.solid(Color.kRed)
            .blink(Seconds.of(0.1))
            .overlayOn(LEDPattern.solid(Color.kBlue));
    public static final LEDPattern fireball =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kMagenta, Color.kBlue)
            .scrollAtRelativeSpeed(Percent.per(Second).of(100))
            .overlayOn(LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(2.0)))
            .blink(Seconds.of(1.0))
            .overlayOn(
                LEDPattern.gradient(
                        LEDPattern.GradientType.kContinuous, Color.kMagenta, Color.kBlue)
                    .scrollAtRelativeSpeed(Percent.per(Second).of(100))
                    .overlayOn(LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(2.0)))
                    .reversed());
  }
}
