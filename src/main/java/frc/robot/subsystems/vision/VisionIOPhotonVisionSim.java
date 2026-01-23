// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
    super(name, robotToCamera);

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);

    // Enable camera streams in simulation
    boolean renderSim = false;
    cameraSim.enableRawStream(renderSim);
    cameraSim.enableProcessedStream(renderSim);
    cameraSim.enableDrawWireframe(renderSim);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    super.updateInputs(inputs);
  }
  /** 
   * Updates the vision simulation with the current robot pose.
   * Needs to be called once per iteration of the main loop.
   */
  public static void updateSim() {
    visionSim.update(DriveSubsystem.getInstance().getPose());
  }
}
