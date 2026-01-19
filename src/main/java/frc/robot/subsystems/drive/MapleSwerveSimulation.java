package frc.robot.subsystems.drive;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class MapleSwerveSimulation {
  private static SwerveDriveSimulation INSTANCE = null;

  private MapleSwerveSimulation() {}

  public static SwerveDriveSimulation getInstance() {
    if (INSTANCE == null) {
      INSTANCE =
          new SwerveDriveSimulation(
              DriveConstants.driveTrainSimulationConfig, DriveConstants.simStartingPose);
      // Register the drivetrain simulation to the default simulation world
      SimulatedArena.getInstance().addDriveTrainSimulation(INSTANCE);
    }
    return INSTANCE;
  }
}
