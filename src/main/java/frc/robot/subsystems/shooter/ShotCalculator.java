package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

  public static ShotParameters calculateShotParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Add whatever you want to do to calculate the shot here
    // Let's assume the robot pose is the same as the turret pose for now, (like the turret is
    // mounted on the center of the robot)
    // It just needs to return a ShotParameters object which will be used to launch the simulated
    // ball

    // isValid, turretAngle (field-relative), hoodAngle (above horizontal), ballExitVelocity (m/s)
    return new ShotParameters(
        true, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(Math.random() * 90), 8.0);
  }

  public static Command launchSimulatedFuel() {
    // Get the drive simulation
    SwerveDriveSimulation driveSimulation = DriveSubsystem.getInstance().getSwerveDriveSimulation();

    // Get the robot pose and field-relative speeds
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    ChassisSpeeds fieldRelativeSpeeds =
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

    return Commands.runOnce(
        () -> {
          // Calculate the shot parameters
          ShotParameters shotParameters = calculateShotParameters(robotPose, fieldRelativeSpeeds);

          // Check if the shot is valid
          if (!shotParameters.isValid) {
            System.out.println("Shot is not valid");
            return;
          }

          // Launch the simulated fuel
          GamePieceProjectile fuel =
              new RebuiltFuelOnFly(
                      // robot position
                      driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                      // shooter offset from center (zero)
                      Translation2d.kZero,
                      // field-relative speed of the robot
                      driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                      // absolute field-relative angle of the turret (you could change this to be
                      // the turret angle relative to the robot)
                      // it would be something like
                      // driveSimulation.getSimulatedDriveTrainPose().getRotation().plus(shotParameters.turretAngle)
                      shotParameters.turretAngle,
                      // initial height of the ball, in meters (assuming shooter is 22 cm above
                      // floor)
                      Units.Meters.of(0.22),
                      // initial velocity, in m/s
                      Units.MetersPerSecond.of(shotParameters.ballExitVelocity),
                      // shooter angle (above horizontal)
                      Units.Radians.of(shotParameters.hoodAngle.getRadians()))
                  .withProjectileTrajectoryDisplayCallBack(
                      // Successful seems to not work properly, all shots get logged as missed
                      // This is probably a maple sim bug
                      (poses) ->
                          Logger.recordOutput(
                              "FieldSimulation/successfulShotsTrajectory",
                              poses.toArray(Pose3d[]::new)),
                      (poses) ->
                          Logger.recordOutput(
                              "FieldSimulation/missedShotsTrajectory",
                              poses.toArray(Pose3d[]::new)));
          fuel.setHitTargetCallBack(() -> System.out.println("FUEL hits HUB!"));

          SimulatedArena.getInstance().addGamePieceProjectile(fuel);
        });
  }

  public record ShotParameters(
      // whether or not we can make the shot
      boolean isValid,
      // absolute angle of the turret relative to the field, CCW positive
      Rotation2d turretAngle,
      // rotation above horizontal
      Rotation2d hoodAngle,
      // velocity of the ball as it leaves the shooter in m/s
      double ballExitVelocity) {}
}
