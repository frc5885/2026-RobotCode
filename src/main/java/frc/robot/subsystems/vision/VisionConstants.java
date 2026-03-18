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

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          -0.19021425,
          -0.08601456,
          0.514521196,
          new Rotation3d(0.0, -Units.degreesToRadians(20.0), 0.0));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          -0.325795386,
          -0.291894514,
          0.423041826,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(170.0)));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.291822124,
          -0.325782686,
          0.423041826,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(280.0)));

  // Should be -X (small-ish number), +Y (large-ish number), +Z (large-ish number)
  // Rotation is facing straight out the left side of the robot
  public static Transform3d robotToCamera3 =
      new Transform3d(
          -0.01075, 0.32925, 0.46304286, new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };
}
