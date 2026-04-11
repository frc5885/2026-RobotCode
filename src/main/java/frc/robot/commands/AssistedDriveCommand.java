// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.ControllerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.ControllerUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;
import frc.robot.util.OverrideUtil;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.Zones;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

/** Default drive command to run that drives based on controller input */
public class AssistedDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  @AutoLogOutput private final Trigger inTrenchZoneTrigger;

  @AutoLogOutput private final Trigger inBumpZoneTrigger;

  @AutoLogOutput private final Trigger inTowerZoneTrigger;

  @AutoLogOutput private final Trigger inHubDropAreaTrigger;

  private final PIDController trenchYController =
      new PIDController(
          DriveConstants.driveAssistTranslationKp,
          DriveConstants.driveAssistTranslationKi,
          DriveConstants.driveAssistTranslationKd);

  private final PIDController towerXController =
      new PIDController(
          DriveConstants.driveAssistTranslationKp,
          DriveConstants.driveAssistTranslationKi,
          DriveConstants.driveAssistTranslationKd);
  private final PIDController hubXController =
      new PIDController(
          DriveConstants.driveAssistTranslationKp,
          DriveConstants.driveAssistTranslationKi,
          DriveConstants.driveAssistTranslationKd);
  private final PIDController rotationController =
      new PIDController(
          DriveConstants.driveAssistRotationKp,
          DriveConstants.driveAssistRotationKi,
          DriveConstants.driveAssistRotationKd);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;

  /** Creates a new TeleopDrive. */
  public AssistedDriveCommand(CommandXboxController controller) {
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();
    this.driveLimiter = new SlewRateLimiter2d(DriveConstants.maxAccelerationMetersPerSec2);

    trenchYController.setTolerance(DriveConstants.trenchAlignPositionTolerance);
    towerXController.setTolerance(DriveConstants.towerAlignPositionTolerance);
    hubXController.setTolerance(DriveConstants.hubAlignPositionTolerance);
    rotationController.setTolerance(DriveConstants.rotationAlignTolerance);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    Trigger notSprinting = ControllerUtil.sprintToggle(controller).negate();
    Trigger noOverridesActive =
        notSprinting
            .and(() -> !DriverStation.isTest())
            .and(OverrideUtil.isManualModeTrigger().negate());

    inTrenchZoneTrigger =
        noOverridesActive.and(
            Zones.trenchZones
                .willContain(
                    driveSubsystem::getPose,
                    driveSubsystem::getFieldRelativeChassisSpeeds,
                    Seconds.of(DriveConstants.trenchAlignTimeSeconds))
                .debounce(0.1));

    inTowerZoneTrigger = new Trigger(() -> false);
    // noOverridesActive.and(
    //     Zones.towerZones
    //         .willContain(
    //             driveSubsystem::getPose,
    //             driveSubsystem::getFieldRelativeChassisSpeeds,
    //             Seconds.of(DriveConstants.towerAlignTimeSeconds))
    //         .debounce(0.1));

    inHubDropAreaTrigger =
        noOverridesActive.and(
            Zones.hubDropAreas
                .willContain(
                    driveSubsystem::getPose,
                    driveSubsystem::getFieldRelativeChassisSpeeds,
                    Seconds.of(DriveConstants.hubDropAreaTimeSeconds))
                .debounce(0.1));

    inBumpZoneTrigger =
        noOverridesActive
            .and(
                Zones.bumpZones
                    .willContain(
                        driveSubsystem::getPose,
                        driveSubsystem::getFieldRelativeChassisSpeeds,
                        Seconds.of(DriveConstants.bumpAlignTimeSeconds))
                    .debounce(0.1))
            // negate hub area trigger so that hub and bump don't fight each other
            .and(inHubDropAreaTrigger.negate());

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    inTowerZoneTrigger.onTrue(updateDriveMode(DriveMode.TOWER_LOCK));
    inHubDropAreaTrigger.onTrue(updateDriveMode(DriveMode.HUB_LOCK));
    inTrenchZoneTrigger
        .or(inBumpZoneTrigger)
        .or(inTowerZoneTrigger)
        .or(inHubDropAreaTrigger)
        .onFalse(updateDriveMode(DriveMode.NORMAL));

    addRequirements(driveSubsystem);

    Zones.logAllZones();
    AutoLogOutputManager.addObject(this);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.controllerDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Translation2d(linearMagnitude, linearDirection);
  }

  private Distance getTrenchY() {
    Pose2d robotPose = driveSubsystem.getPose();
    if (robotPose.getMeasureY().gte(Meters.of(FieldConstants.fieldWidth / 2))) {
      return Meters.of(FieldConstants.LeftTrench.center.getY());
    }
    return Meters.of(FieldConstants.RightTrench.center.getY());
  }

  private Rotation2d getTrenchLockAngle() {
    return GeometryUtil.getNearest180Rotation(driveSubsystem.getRotation());
  }

  private Distance getTowerX() {
    if (driveSubsystem.getPose().getX() < FieldConstants.fieldLength / 2) {
      return Meters.of(FieldConstants.Tower.depth / 2);
    }
    return Meters.of(FieldConstants.fieldLength - FieldConstants.Tower.depth / 2);
  }

  private Distance getHubX() {
    if (driveSubsystem.getPose().getX() < FieldConstants.fieldLength / 2) {
      return Meters.of(FieldConstants.LinesVertical.hubCenter + FieldConstants.Hub.width);
    }
    return Meters.of(FieldConstants.LinesVertical.oppHubCenter - FieldConstants.Hub.width);
  }

  private Rotation2d getTowerLockAngle() {
    return GeometryUtil.getNearest90or270Rotation(driveSubsystem.getRotation());
  }

  private Rotation2d getHubLockAngle() {
    return GeometryUtil.getNearest90or270Rotation(driveSubsystem.getRotation());
  }

  private Rotation2d getBumpLockAngle() {
    // Angles where the robot's intake (front, 0°) faces in the ±X direction
    Rotation2d[] intakeFirstAngles = {Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-45)};
    // Angles where the robot's intake faces away from the ±X direction (intake trails)
    Rotation2d[] intakeTrailingAngles = {Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-135)};

    boolean isIntakeDown = IntakeSubsystem.getInstance().isExtensionDown();
    if (!isIntakeDown) {
      return GeometryUtil.getNearestRotation(
          driveSubsystem.getRotation(),
          intakeFirstAngles[0],
          intakeFirstAngles[1],
          intakeTrailingAngles[0],
          intakeTrailingAngles[1]);
    }

    // Intake is down — only allow headings where the intake trails the direction of X travel,
    // since we cannot drive over the bump intake-first when it is extended
    double vx = driveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond;
    if (vx > 0) {
      // Moving in +X: intake must face -X → 135° or -135°
      return GeometryUtil.getNearestRotation(
          driveSubsystem.getRotation(), intakeTrailingAngles[0], intakeTrailingAngles[1]);
    } else if (vx < 0) {
      // Moving in -X: intake must face +X → 45° or -45°
      return GeometryUtil.getNearestRotation(
          driveSubsystem.getRotation(), intakeFirstAngles[0], intakeFirstAngles[1]);
    } else {
      // No X motion; any heading is fine
      return GeometryUtil.getNearestRotation(
          driveSubsystem.getRotation(),
          intakeFirstAngles[0],
          intakeFirstAngles[1],
          intakeTrailingAngles[0],
          intakeTrailingAngles[1]);
    }
  }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(
        () -> {
          currentDriveMode = driveMode;
          rotationController.reset();
          trenchYController.reset();
          towerXController.reset();
          hubXController.reset();
        });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDriveMode = DriveMode.NORMAL;
    trenchYController.reset();
    rotationController.reset();
    driveLimiter.reset(new Translation2d());
    flipFactor =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? -1
            : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearVelocity = linearVelocity.times(driveSubsystem.getMaxLinearSpeedMetersPerSec());
    linearVelocity = driveLimiter.calculate(linearVelocity);

    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.controllerDeadband);
    omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

    switch (currentDriveMode) {
      case NORMAL:
        driveSubsystem.runVelocityFieldRelative(
            new ChassisSpeeds(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(linearVelocity.getY()),
                RadiansPerSecond.of(driveSubsystem.getMaxAngularSpeedRadPerSec()).times(omega)));
        break;
      case TRENCH_LOCK:
        trenchYController.setSetpoint(getTrenchY().in(Meters));
        // Clamp the y velocity to the max linear speed
        double yVel =
            MathUtil.clamp(
                trenchYController.calculate(driveSubsystem.getPose().getY()),
                -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                driveSubsystem.getMaxLinearSpeedMetersPerSec());
        if (trenchYController.atSetpoint()) {
          yVel = 0;
        }
        rotationController.setSetpoint(getTrenchLockAngle().getRadians());
        // Clamp the rotation speed to the max angular speed
        double rotSpeedToStraight =
            MathUtil.clamp(
                rotationController.calculate(driveSubsystem.getRotation().getRadians()),
                -driveSubsystem.getMaxAngularSpeedRadPerSec(),
                driveSubsystem.getMaxAngularSpeedRadPerSec());
        if (rotationController.atSetpoint()) {
          rotSpeedToStraight = 0;
        }
        driveSubsystem.runVelocityFieldRelative(
            new ChassisSpeeds(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(yVel),
                RadiansPerSecond.of(rotSpeedToStraight)));
        break;
      case BUMP_LOCK:
        rotationController.setSetpoint(getBumpLockAngle().getRadians());
        // Clamp the rotation speed to the max angular speed
        double rotSpeedToDiagonal =
            MathUtil.clamp(
                rotationController.calculate(driveSubsystem.getRotation().getRadians()),
                -driveSubsystem.getMaxAngularSpeedRadPerSec(),
                driveSubsystem.getMaxAngularSpeedRadPerSec());
        if (rotationController.atSetpoint()) {
          rotSpeedToDiagonal = 0;
        }
        driveSubsystem.runVelocityFieldRelative(
            new ChassisSpeeds(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(linearVelocity.getY()),
                RadiansPerSecond.of(rotSpeedToDiagonal)));
        break;

      case TOWER_LOCK:
        towerXController.setSetpoint(getTowerX().in(Meters));
        double xVel =
            MathUtil.clamp(
                towerXController.calculate(driveSubsystem.getPose().getX()),
                -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                driveSubsystem.getMaxLinearSpeedMetersPerSec());
        if (towerXController.atSetpoint()) {
          xVel = 0;
        }
        rotationController.setSetpoint(getTowerLockAngle().getRadians());
        double rotSpeedToSideways =
            MathUtil.clamp(
                rotationController.calculate(driveSubsystem.getRotation().getRadians()),
                -driveSubsystem.getMaxAngularSpeedRadPerSec(),
                driveSubsystem.getMaxAngularSpeedRadPerSec());
        if (rotationController.atSetpoint()) {
          rotSpeedToSideways = 0;
        }
        driveSubsystem.runVelocityFieldRelative(
            new ChassisSpeeds(
                MetersPerSecond.of(xVel),
                MetersPerSecond.of(linearVelocity.getY()),
                RadiansPerSecond.of(rotSpeedToSideways)));
        break;

      case HUB_LOCK:
        hubXController.setSetpoint(getHubX().in(Meters));
        double xHubVel =
            MathUtil.clamp(
                hubXController.calculate(driveSubsystem.getPose().getX()),
                -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                driveSubsystem.getMaxLinearSpeedMetersPerSec());
        if (hubXController.atSetpoint()) {
          xHubVel = 0;
        }
        rotationController.setSetpoint(getHubLockAngle().getRadians());
        double rotSpeedToSidewaysHub =
            MathUtil.clamp(
                rotationController.calculate(driveSubsystem.getRotation().getRadians()),
                -driveSubsystem.getMaxAngularSpeedRadPerSec(),
                driveSubsystem.getMaxAngularSpeedRadPerSec());
        if (rotationController.atSetpoint()) {
          rotSpeedToSidewaysHub = 0;
        }
        driveSubsystem.runVelocityFieldRelative(
            new ChassisSpeeds(
                MetersPerSecond.of(xHubVel),
                MetersPerSecond.of(linearVelocity.getY()),
                RadiansPerSecond.of(rotSpeedToSidewaysHub)));

        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    TOWER_LOCK,
    HUB_LOCK
  }
}
