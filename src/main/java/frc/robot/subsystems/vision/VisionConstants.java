// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "kowalski";
  public static String camera1Name = "private";
  public static String camera2Name = "skipper";
  public static String camera3Name = "rico";
  public static String camera4Name = "kingJulien";

  // Index of the game piece detection camera
  public static final int gamePieceCameraIndex = 4;

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          -0.325795 + Units.inchesToMeters(1.0),
          -0.291894 + Units.inchesToMeters(10.0),
          0.423041 + Units.inchesToMeters(4.0),
          new Rotation3d(0.0, -Units.degreesToRadians(20.0), 0.0));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          -0.325795, -0.291894, 0.423041, new Rotation3d(0.0, 0.0, Units.degreesToRadians(170.0)));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.291822, -0.325782, 0.423041, new Rotation3d(0.0, 0.0, Units.degreesToRadians(280.0)));

  public static Transform3d robotToCamera3 =
      new Transform3d(
          -0.01075, 0.32925, 0.46304286, new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.5; // Meters
  public static double angularStdDevBaseline = Units.degreesToRadians(60.0); // Radians

  // Bump zone vision boost: temporarily trust vision more after crossing the bump
  // to speed up pose convergence after odometry corruption from airtime
  public static double bumpBoostFactor =
      0.1; // Std dev multiplier when boosted (lower = more trust)
  public static double bumpBoostDuration = 2.0; // Seconds of boost after exiting bump zone

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // 1.0 = no change, 2.0 = 2x worse (trust 1/2 as much), 0.5 = 2x better (trust 2x more)
  public static double[] cameraStdDevFactors =
      new double[] {
        // DO NOT doubt kowalski
        0.7, // Camera 0 (we ALWAYS trust big K)
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };
}
