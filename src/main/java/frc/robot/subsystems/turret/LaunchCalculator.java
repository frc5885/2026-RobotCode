// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator {
  private static volatile LaunchCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.dtSeconds));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.dtSeconds));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle = Double.NaN;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  private LaunchMode launchMode = LaunchMode.SHOOTING;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.322;
    maxDistance = 6.265;
    phaseDelay = 0.03;

    launchHoodAngleMap.put(1.322, Rotation2d.fromDegrees(80.0));
    launchHoodAngleMap.put(1.712, Rotation2d.fromDegrees(80.0));
    launchHoodAngleMap.put(2.015, Rotation2d.fromDegrees(78.0));
    launchHoodAngleMap.put(2.686, Rotation2d.fromDegrees(75.0));
    launchHoodAngleMap.put(3.479, Rotation2d.fromDegrees(70.0));
    launchHoodAngleMap.put(3.896, Rotation2d.fromDegrees(70.0));
    launchHoodAngleMap.put(4.280, Rotation2d.fromDegrees(68.0));
    launchHoodAngleMap.put(4.664, Rotation2d.fromDegrees(67.0));
    launchHoodAngleMap.put(5.086, Rotation2d.fromDegrees(65.0));
    launchHoodAngleMap.put(5.449, Rotation2d.fromDegrees(65.0));
    launchHoodAngleMap.put(5.886, Rotation2d.fromDegrees(65.0));
    launchHoodAngleMap.put(6.265, Rotation2d.fromDegrees(64.0));

    launchFlywheelSpeedMap.put(1.322, Units.rotationsPerMinuteToRadiansPerSecond(1800));
    launchFlywheelSpeedMap.put(1.712, Units.rotationsPerMinuteToRadiansPerSecond(2056));
    launchFlywheelSpeedMap.put(2.015, Units.rotationsPerMinuteToRadiansPerSecond(2056));
    launchFlywheelSpeedMap.put(2.686, Units.rotationsPerMinuteToRadiansPerSecond(2100));
    launchFlywheelSpeedMap.put(3.479, Units.rotationsPerMinuteToRadiansPerSecond(2200));
    launchFlywheelSpeedMap.put(3.896, Units.rotationsPerMinuteToRadiansPerSecond(2350));
    launchFlywheelSpeedMap.put(4.280, Units.rotationsPerMinuteToRadiansPerSecond(2400));
    launchFlywheelSpeedMap.put(4.664, Units.rotationsPerMinuteToRadiansPerSecond(2400));
    launchFlywheelSpeedMap.put(5.086, Units.rotationsPerMinuteToRadiansPerSecond(2500));
    launchFlywheelSpeedMap.put(5.449, Units.rotationsPerMinuteToRadiansPerSecond(2550));
    launchFlywheelSpeedMap.put(5.886, Units.rotationsPerMinuteToRadiansPerSecond(2650));
    launchFlywheelSpeedMap.put(6.265, Units.rotationsPerMinuteToRadiansPerSecond(2800));

    timeOfFlightMap.put(1.544, 1.042);
    timeOfFlightMap.put(2.468, 1.096);
    timeOfFlightMap.put(3.542, 1.143);
    timeOfFlightMap.put(4.390, 1.233);
    timeOfFlightMap.put(5.196, 1.292);
    timeOfFlightMap.put(6.177, 1.439);
  }

  public LaunchingParameters getParameters() {
    // Parameters get cleared once per period in robotPeriodic
    // Subsystems use cached parameters to avoid redundant calculations
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = DriveSubsystem.getInstance().getPose();

    ChassisSpeeds robotRelativeVelocity = DriveSubsystem.getInstance().getChassisSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    if (AllianceFlipUtil.applyX(estimatedPose.getX()) <= FieldConstants.LinesVertical.hubCenter) {
      launchMode = LaunchMode.SHOOTING;
    } else {
      launchMode = LaunchMode.PASSING;
    }

    // Calculate distance from turret to target
    Translation2d target = FieldConstants.getTurretTarget(estimatedPose);
    Logger.recordOutput("LaunchCalculator/Target", target);
    Pose2d turretPosition =
        estimatedPose.transformBy(
            new Transform2d(
                TurretConstants.robotToTurret.getTranslation().toTranslation2d(),
                TurretConstants.robotToTurret.getRotation().toRotation2d()));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = DriveSubsystem.getInstance().getFieldRelativeChassisSpeeds();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (-TurretConstants.robotToTurret.getX() * Math.sin(robotAngle)
                    - TurretConstants.robotToTurret.getY() * Math.cos(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.robotToTurret.getX() * Math.cos(robotAngle)
                    - TurretConstants.robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = getTimeOfFlight(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = getHoodAngle(lookaheadTurretToTargetDistance);
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.dtSeconds);
    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.dtSeconds);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    boolean isValid = checkIfValid(lookaheadTurretToTargetDistance);
    double flywheelSpeed = getFlywheelSpeed(lookaheadTurretToTargetDistance);
    latestParameters =
        new LaunchingParameters(
            isValid, turretAngle, turretVelocity, hoodAngle, hoodVelocity, flywheelSpeed);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput("LaunchCalculator/IsValid", isValid);
    Logger.recordOutput("LaunchCalculator/TurretAngle", turretAngle);
    Logger.recordOutput("LaunchCalculator/HoodAngle", hoodAngle);
    Logger.recordOutput("LaunchCalculator/TurretVelocity", turretVelocity);
    Logger.recordOutput("LaunchCalculator/HoodVelocity", hoodVelocity);
    Logger.recordOutput("LaunchCalculator/FlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("LaunchCalculator/LaunchMode", launchMode);

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public enum LaunchMode {
    SHOOTING,
    PASSING
  }

  private double getHoodAngle(double distance) {
    switch (launchMode) {
      case SHOOTING:
        return launchHoodAngleMap.get(distance).getRadians();
      default:
        // Probably need to tune this
        return Math.toRadians((0.178 * Math.pow(distance - 13.34, 2)) + 45.0);
    }
  }

  private double getFlywheelSpeed(double distance) {
    switch (launchMode) {
      case SHOOTING:
        return launchFlywheelSpeedMap.get(distance);
      default:
        // Probably need to tune this
        return ((0.903 * Math.pow(distance, 2)) + (10.68 * distance) + 194.0) * 0.8;
    }
  }

  private double getTimeOfFlight(double distance) {
    switch (launchMode) {
      case SHOOTING:
        return timeOfFlightMap.get(distance);
      default:
        // Probably need to tune this
        return ((0.043856 * distance) + 0.93);
    }
  }

  private boolean checkIfValid(double distance) {
    switch (launchMode) {
      case SHOOTING:
        return distance >= minDistance && distance <= maxDistance;
      default:
        // Passing is always valid
        return true;
    }
  }

  public LaunchMode getLaunchMode() {
    return launchMode;
  }

  public static double getMinTimeOfFlight() {
    return 0.0;
  }

  public static double getMaxTimeOfFlight() {
    return 1.6;
  }
}
