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
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
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

  public enum LEDState {
    DISABLED(Patterns.rainbow, 0, false),
    IDLE(Patterns.fireball, 1, false),
    INTAKE_RUNNING(Patterns.intakeRunning, 5, false),
    AIMING(Patterns.cleanRed, 10, false),
    SHOOTING(Patterns.cleanGreen, 11, false),
    MANUAL_MODE(Patterns.manualMode, 99, true),
    SHIFT_CHANGE(Patterns.shiftChange, 100, true),
    BOGUS_CALL(Patterns.policeSirens, 999, false),

    TEST_PATTERN(Patterns.eric, 1000, false);

    public final LEDPattern pattern;
    public final int priority;
    public final boolean isOverlay;

    LEDState(LEDPattern pattern, int priority, boolean isOverlay) {
      this.pattern = pattern;
      this.priority = priority;
      this.isOverlay = isOverlay;
    }
  }

  private static final class Patterns {
    private static final Distance ledSpacing = Meters.of(1.0 / 60);
    private static final LinearVelocity scrollSpeed = InchesPerSecond.of(6.0);

    private static final LEDPattern rainbow =
        LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(scrollSpeed, ledSpacing);
    private static final LEDPattern intakeRunning =
        LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.2));
    private static final LEDPattern cyanScrollingGradient =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kBlack)
            .scrollAtRelativeSpeed(Percent.per(Second).of(100.0));
    private static final LEDPattern eric =
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
    private static final LEDPattern policeSirens =
        LEDPattern.solid(Color.kRed)
            .blink(Seconds.of(0.1))
            .overlayOn(LEDPattern.solid(Color.kBlue));

    private static final Frequency fireballSpeed = Percent.per(Second).of(100);

    private static final LEDPattern fireball =
        LEDPattern.steps(Map.of(0, Color.kRed, 0.05, Color.kBlack))
            .scrollAtRelativeSpeed(fireballSpeed)
            .overlayOn(
                (LEDPattern.gradient(
                            GradientType.kDiscontinuous,
                            Color.kBlack,
                            Color.kBlack,
                            Color.kBlack,
                            Color.kBlack,
                            Color.kMagenta,
                            Color.kRed)
                        .scrollAtRelativeSpeed(fireballSpeed)
                        .blend(
                            LEDPattern.gradient(
                                    GradientType.kDiscontinuous,
                                    Color.kBlack,
                                    Color.kBlack,
                                    Color.kBlack,
                                    Color.kBlack,
                                    Color.kMagenta,
                                    Color.kRed)
                                .scrollAtRelativeSpeed(fireballSpeed)))
                    .blend(
                        LEDPattern.gradient(
                                GradientType.kDiscontinuous,
                                Color.kBlack,
                                Color.kBlack,
                                Color.kBlack,
                                Color.kBlack,
                                Color.kBlack,
                                Color.kRed)
                            .scrollAtRelativeSpeed(fireballSpeed)
                            .blend(
                                LEDPattern.gradient(
                                        GradientType.kDiscontinuous,
                                        Color.kBlack,
                                        Color.kBlack,
                                        Color.kBlack,
                                        Color.kMagenta,
                                        Color.kMagenta,
                                        Color.kRed)
                                    .scrollAtRelativeSpeed(fireballSpeed))));

    private static final LEDPattern manualMode =
        LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.5), Seconds.of(1.5));
    private static final LEDPattern redBreathe =
        LEDPattern.solid(Color.kRed).breathe(Seconds.of(1.0));
    private static final LEDPattern greenBreathe =
        LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.2));
    private static final LEDPattern shiftChange =
        LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.05));

    private static final LEDPattern cleanGreen =
        LEDPattern.steps(Map.of(0.95, Color.kLimeGreen))
            .scrollAtRelativeSpeed(Percent.per(Second).of(100.0))
            .overlayOn(
                LEDPattern.steps(Map.of(0.95, Color.kLimeGreen))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(100.0))
                    .reversed())
            .overlayOn(LEDPattern.solid(Color.kGreen).breathe(Seconds.of(1.0)));

    private static final LEDPattern cleanRed =
        LEDPattern.steps(Map.of(0.95, Color.kRed))
            .scrollAtRelativeSpeed(Percent.per(Second).of(100.0))
            .overlayOn(
                LEDPattern.steps(Map.of(0.95, Color.kRed))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(100.0))
                    .reversed())
            .overlayOn(LEDPattern.solid(Color.kDarkRed).breathe(Seconds.of(1.0)));
  }
}
