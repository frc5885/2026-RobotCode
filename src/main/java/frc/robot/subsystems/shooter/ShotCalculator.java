package frc.robot.subsystems.shooter;

import static java.lang.Math.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FieldConstants;
import java.io.File;
import java.io.IOException;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

  ObjectMapper objectMapper = new ObjectMapper();

  File file = new File("trajectorySampleCalcs.json");

  // loads the lookup table from the JSON file
  private static int[][][] loadLookupTable() {
    ObjectMapper mapper = new ObjectMapper();
    try {
      return mapper.readValue(
          new File("src\\main\\java\\frc\\robot\\subsystems\\shooter\\trajectorySampleCalcs.json"),
          int[][][].class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to load lookup table", e);
    }
  }

  private record AimData(double turretAngle, double launchAngle, double velocity) {}

  // creates the lookup table for the sample trajectory calculations
  private static final int[][][] sampleCalcsTable = loadLookupTable();

  // decompresses the data into separate variables
  private static AimData decompressData(int data) {
    // 32 bit integer limit is used for null values
    if (data == Integer.MAX_VALUE) return null;

    // first four digits are used to store turret angle
    double turretAngle = (data / 1000000) / 10.0;
    data -= (data / 1000000) * 1000000;
    data = abs(data);

    // next three digits are used to store hood angle
    double launchAngle = (data / 1000) / 10.0;
    data -= (data / 1000) * 1000;

    // last three digits are used to store velocity
    double velocity = data / 10.0;

    return new AimData(turretAngle, launchAngle, velocity);
  }

  // trilinear interpolation method

  private static double trilinearInterpolation(
      double c000,
      double c100,
      double c010,
      double c110,
      double c001,
      double c101,
      double c011,
      double c111,
      double distanceWeight,
      double angleWeight,
      double velocityWeight) {

    double c00 = c000 * (1 - distanceWeight) + c100 * distanceWeight;
    double c10 = c010 * (1 - distanceWeight) + c110 * distanceWeight;
    double c01 = c001 * (1 - distanceWeight) + c101 * distanceWeight;
    double c11 = c011 * (1 - distanceWeight) + c111 * distanceWeight;

    double c0 = c00 * (1 - angleWeight) + c10 * angleWeight;
    double c1 = c01 * (1 - angleWeight) + c11 * angleWeight;

    return c0 * (1 - velocityWeight) + c1 * velocityWeight;
  }

  public static ShotParameters calculateShotParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Add whatever you want to do to calculate the shot here
    // Let's assume the robot pose is the same as the turret pose for now, (like the turret is
    // mounted on the center of the robot)
    // It just needs to return a ShotParameters object which will be used to launch the simulated
    // ball

    Translation3d hubPose = FieldConstants.Hub.topCenterPoint;

    // distance and angle from the hub
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();

    double distance = hypot(dx, dy);
    double HUBangle = atan2(dy, dx);

    // gets robot velocity
    double vx = fieldRelativeSpeeds.vxMetersPerSecond;
    double vy = fieldRelativeSpeeds.vyMetersPerSecond;

    double velocityMagnitude = hypot(vx, vy);
    double velocityAngle = atan2(vy, vx);

    // calculates angle relative to the distance vector
    double angle = (velocityAngle - HUBangle) * 180 / PI;

    // converts the raw values to theoretical indices
    double dIndex = (distance - 1.0) / 0.5;
    double vIndex = velocityMagnitude / 0.25;
    double aIndex = (angle + 180) / 15.0;

    // lower bound indices
    int d0 = (int) dIndex;
    int v0 = (int) vIndex;
    int a0 = (int) aIndex;

    // upper bound indices
    int d1 = d0 + 1;
    int v1 = v0 + 1;
    int a1 = (a0 < sampleCalcsTable[0].length - 1) ? a0 + 1 : 0;

    // clamp to bounds
    d1 = Math.min(d1, sampleCalcsTable.length - 1);
    v1 = Math.min(v1, sampleCalcsTable[0][0].length - 1);

    // blended weights for interpolation
    double distanceWeight = dIndex - d0;
    double velocityWeight = vIndex - v0;
    double angleWeight = aIndex - a0;

    // declares these variables
    AimData c000, c100, c010, c110, c001, c101, c011, c111;

    // try/catch block to handle array index out of bounds exceptions
    try {
      // gets the aim data for all eight combinations of indices
      c000 = decompressData(sampleCalcsTable[d0][a0][v0]);
      c001 = decompressData(sampleCalcsTable[d0][a0][v1]);
      c010 = decompressData(sampleCalcsTable[d0][a1][v0]);
      c011 = decompressData(sampleCalcsTable[d0][a1][v1]);

      c100 = decompressData(sampleCalcsTable[d1][a0][v0]);
      c101 = decompressData(sampleCalcsTable[d1][a0][v1]);
      c110 = decompressData(sampleCalcsTable[d1][a1][v0]);
      c111 = decompressData(sampleCalcsTable[d1][a1][v1]);

    } catch (ArrayIndexOutOfBoundsException e) {
      return new ShotParameters(
          false, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(Math.random() * 90), 8.0);
    }

    // returns null if any of the values are null
    if (c000 == null
        || c100 == null
        || c010 == null
        || c110 == null
        || c001 == null
        || c101 == null
        || c011 == null
        || c111 == null) {
      return new ShotParameters(
          false, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(Math.random() * 90), 8.0);
    }

    // performs trilinear interpolation to calculate the turret angle
    double turretAngle =
        trilinearInterpolation(
                c000.turretAngle,
                c100.turretAngle,
                c010.turretAngle,
                c110.turretAngle,
                c001.turretAngle,
                c101.turretAngle,
                c011.turretAngle,
                c111.turretAngle,
                distanceWeight,
                angleWeight,
                velocityWeight)
            + toDegrees(HUBangle);

    // performs trilinear interpolation to calculate the hood angle
    double launchAngle =
        trilinearInterpolation(
            c000.launchAngle,
            c100.launchAngle,
            c010.launchAngle,
            c110.launchAngle,
            c001.launchAngle,
            c101.launchAngle,
            c011.launchAngle,
            c111.launchAngle,
            distanceWeight,
            angleWeight,
            velocityWeight);

    // performs trilinear interpolation to calculate the fuel velocity
    double fuelVelocity =
        trilinearInterpolation(
            c000.velocity,
            c100.velocity,
            c010.velocity,
            c110.velocity,
            c001.velocity,
            c101.velocity,
            c011.velocity,
            c111.velocity,
            distanceWeight,
            angleWeight,
            velocityWeight);

    // isValid, turretAngle (field-relative), hoodAngle (above horizontal), ballExitVelocity (m/s)
    return new ShotParameters(
        true,
        Rotation2d.fromDegrees(turretAngle),
        Rotation2d.fromDegrees(launchAngle),
        fuelVelocity);
  }

  public static Command launchSimulatedFuel() {
    // Get the drive simulation
    SwerveDriveSimulation driveSimulation = DriveSubsystem.getInstance().getSwerveDriveSimulation();

    return Commands.runOnce(
        () -> {
          // Get the robot pose and field-relative speeds at execution time
          Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
          ChassisSpeeds fieldRelativeSpeeds =
              driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

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
                      Units.Radians.of(shotParameters.launchAngle.getRadians()))
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
      Rotation2d launchAngle,
      // velocity of the ball as it leaves the shooter in m/s
      double ballExitVelocity) {}
}
