package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Translation3d;

public class HopperConstants {
  public static final int hopperCapacity = 30;

  // Hopper bowl center position (robot-relative, meters)
  public static final Translation3d robotRelativeHopperCenter =
      new Translation3d(0.12, -0.045, 0.15);

  // For sim visualization
  public static class Sim {
    // Ball ring layout — 6 balls per layer, 5 layers = 30 ball capacity
    public static final int ballsPerRing = 6;
    public static final int maxRingLayers = 5;
    public static final double ballDiameter = 0.075; // meters (~3 inches)
    public static final double ballRingRadius = 0.12; // ring radius around hopper center
  }
}
