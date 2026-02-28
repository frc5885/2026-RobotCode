// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.1; // Tuned on poseidon
  public static final double shootOnTheMoveSpeedMultiplier = 0.15;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(24.25);
  public static final double wheelBase = Units.inchesToMeters(24.25);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };
  // Used for maple sim
  public static final double robotLength = Units.inchesToMeters(27.5);
  public static final double robotWidth = Units.inchesToMeters(27.5);
  public static final double bumperWidth = Units.inchesToMeters(3.0);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(2.11);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-3.11);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.067);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(2.53);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 13;
  public static final int backLeftDriveCanId = 10;
  public static final int frontRightDriveCanId = 12;
  public static final int backRightDriveCanId = 11;

  public static final int frontLeftTurnCanId = 23;
  public static final int backLeftTurnCanId = 20;
  public static final int frontRightTurnCanId = 22;
  public static final int backRightTurnCanId = 21;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.944);
  public static final double driveMotorReduction = (6.75) / (1.0); // SDS Mk4i L2
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0; // Don't use kP or kD
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.16493; // Tuned on poseidon
  public static final double driveKv = 0.13084; // Tuned on poseidon
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.10819; // Tuned with maple sim
  public static final double driveSimKv = 0.15890; // Tuned with maple sim

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = (150.0 / 7.0) / 1.0; // SDS Mk4i
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec
  public static final boolean absoluteEncoderInverted = false;
  public static final double absoluteEncoderPositionFactor =
      (2 * Math.PI) / 3.3; // Volts -> Radians
  public static final double absoluteEncoderVelocityFactor =
      (2 * Math.PI) / 3.3; // V/Sec -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
  public static final double turnPIDToleranceRad = Units.degreesToRadians(1.0);

  // PathPlanner configuration
  public static final double robotMassKg = Units.lbsToKilograms(130.0);
  public static final double robotMOI = 6.8;
  public static final double wheelCOF = COTS.WHEELS.COLSONS.cof;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final PIDConstants pathplannerDrivePID = new PIDConstants(6.0, 0.0, 0.6);
  public static final PIDConstants pathplannerTurnPID = new PIDConstants(6.0, 0.0, 1.0);

  public static final PathConstraints pathConstraints = new PathConstraints(4.1, 8.0, 8.8, 16.0);

  // Create and configure a drivetrain simulation configuration
  public static final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofNav2X())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getNEO(1), // Drive motor is a NEO
                  DCMotor.getNEO(1), // Steer motor is a NEO
                  COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                  2)) // L2 Gear ratio
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(Meters.of(wheelBase), Meters.of(trackWidth))
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(
              Meters.of(robotLength + 2 * bumperWidth), Meters.of(robotWidth + 2 * bumperWidth))
          // Configures the robot mass (for realistic dynamics)
          .withRobotMass(
              Kilograms.of(robotMassKg * 0.5)); // Multiplied by 0.5 to match real robot speed

  public static final Pose2d simStartingPose = new Pose2d(3, 3, new Rotation2d());
}
