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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.Zones;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Default drive command to run that drives based on controller input */
public class AssistedDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  private LinearVelocity maxDriveSpeed =
      MetersPerSecond.of(driveSubsystem.getMaxLinearSpeedMetersPerSec());
  private AngularVelocity maxRotSpeed =
      RadiansPerSecond.of(driveSubsystem.getMaxAngularSpeedRadPerSec());

  @AutoLogOutput private final Trigger inTrenchZoneTrigger;

  @AutoLogOutput private final Trigger inBumpZoneTrigger;

  private final PIDController trenchYController =
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

    inTrenchZoneTrigger =
        Zones.TRENCH_ZONES
            .willContain(
                driveSubsystem::getPose,
                driveSubsystem::getFieldRelativeChassisSpeeds,
                Seconds.of(DriveConstants.trenchAlignTimeSeconds))
            .debounce(0.1);

    inBumpZoneTrigger =
        Zones.BUMP_ZONES
            .willContain(
                driveSubsystem::getPose,
                driveSubsystem::getFieldRelativeChassisSpeeds,
                Seconds.of(DriveConstants.bumpAlignTimeSeconds))
            .debounce(0.1);

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(updateDriveMode(DriveMode.NORMAL));

    addRequirements(driveSubsystem);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.controllerDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
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

  private Rotation2d getBumpLockAngle() {
    for (int i = -135; i < 180; i += 90) {
      if (Math.abs(MathUtil.inputModulus(driveSubsystem.getRotation().getDegrees() - i, -180, 180))
          <= 45) {
        return Rotation2d.fromDegrees(i);
      }
    }
    return Rotation2d.kZero;
  }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(() -> currentDriveMode = driveMode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    linearVelocity = linearVelocity.times(maxDriveSpeed.in(MetersPerSecond));
    linearVelocity = driveLimiter.calculate(linearVelocity);

    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.controllerDeadband);
    omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

    switch (currentDriveMode) {
      case NORMAL:
        driveSubsystem.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(linearVelocity.getY()),
                maxRotSpeed.times(omega),
                driveSubsystem.getRotation()));
        break;
      case TRENCH_LOCK:
        trenchYController.setSetpoint(getTrenchY().in(Meters));
        double yVel = trenchYController.calculate(driveSubsystem.getPose().getY());
        if (trenchYController.atSetpoint()) {
          yVel = 0;
        }
        rotationController.setSetpoint(getTrenchLockAngle().getRadians());
        double rotSpeedToStraight =
            rotationController.calculate(driveSubsystem.getRotation().getRadians());
        if (rotationController.atSetpoint()) {
          rotSpeedToStraight = 0;
        }
        driveSubsystem.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(yVel),
                RadiansPerSecond.of(rotSpeedToStraight),
                driveSubsystem.getRotation()));
        break;
      case BUMP_LOCK:
        rotationController.setSetpoint(getBumpLockAngle().getRadians());
        double rotSpeedToDiagonal =
            rotationController.calculate(driveSubsystem.getRotation().getRadians());
        if (rotationController.atSetpoint()) {
          rotSpeedToDiagonal = 0;
        }
        driveSubsystem.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(linearVelocity.getY()),
                RadiansPerSecond.of(rotSpeedToDiagonal),
                driveSubsystem.getRotation()));
        break;
    }
  }

  private void setDriveSpeed(LinearVelocity speed) {
    maxDriveSpeed = speed;
  }

  private void setRotSpeed(AngularVelocity speed) {
    maxRotSpeed = speed;
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
    BUMP_LOCK
  }
}
