// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class TeleopDrive extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  private LinearVelocity maxDriveSpeed = DriveConstants.defaultDriveSpeed;
  private AngularVelocity maxRotSpeed = DriveConstants.defaultRotationSpeed;

  @AutoLogOutput
  private final Trigger inTrenchZoneTrigger = new Trigger(this::inTrenchZone).debounce(0.1);

  @AutoLogOutput
  private final Trigger inBumpZoneTrigger = new Trigger(this::inBumpZone).debounce(0.1);

  private final PIDController trenchYController =
      // new PIDController(DriveConstants.trenchTranslationConstants);
      new PIDController(5.0, 0.0, 2.0);
  private final PIDController rotationController =
      // new PIDController(DriveConstants.rotationConstants);
      new PIDController(5.0, 0.0, 2.0);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();
    double accelLimit = DriveConstants.maxTeleopAcceleration.in(MetersPerSecondPerSecond);
    this.xLimiter = new SlewRateLimiter(accelLimit);
    this.yLimiter = new SlewRateLimiter(accelLimit);

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(updateDriveMode(DriveMode.NORMAL));
    for (int i = 0; i < 4; i++) {
      Logger.recordOutput("Trench" + i, FieldConstants.trenchZones[i]);
      Logger.recordOutput("Bump" + i, FieldConstants.bumpZones[i]);
    }
    addRequirements(driveSubsystem);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.CONTROLLER_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private boolean inTrenchZone() {
    Pose2d robotPose = driveSubsystem.getPose();
    for (Translation2d[] zone : FieldConstants.trenchZones) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
  }

  private Distance getTrenchY() {
    Pose2d robotPose = driveSubsystem.getPose();
    if (robotPose.getMeasureY().gte(Meters.of(FieldConstants.fieldWidth).div(2))) {
      return Meters.of(FieldConstants.fieldWidth).minus(FieldConstants.trenchCenter);
    }
    return FieldConstants.trenchCenter;
  }

  private Rotation2d getTrenchLockAngle() {
    if (Math.abs(MathUtil.inputModulus(driveSubsystem.getRotation().getDegrees() - 90, -180, 180))
        < 90) {
      return Rotation2d.kCCW_90deg;
    } else {
      return Rotation2d.kCW_90deg;
    }
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

  private boolean inBumpZone() {
    Pose2d robotPose = driveSubsystem.getPose();
    for (Translation2d[] zone : FieldConstants.bumpZones) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
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
    linearVelocity =
        new Translation2d(
            xLimiter.calculate(linearVelocity.getX()), yLimiter.calculate(linearVelocity.getY()));

    switch (currentDriveMode) {
      case NORMAL:
        double omega =
            MathUtil.applyDeadband(
                omegaSupplier.getAsDouble(), ControllerConstants.CONTROLLER_DEADBAND);
        omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

        driveSubsystem.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            maxRotSpeed.times(omega));
        break;
      case TRENCH_LOCK:
        double yVel =
            trenchYController.calculate(driveSubsystem.getPose().getY(), getTrenchY().in(Meters));
        double rotSpeedToStraight =
            rotationController.calculate(
                driveSubsystem.getRotation().getRadians(), getTrenchLockAngle().getRadians());
        driveSubsystem.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(yVel),
            RadiansPerSecond.of(rotSpeedToStraight));
        break;
      case BUMP_LOCK:
        double rotSpeedToDiagonal =
            rotationController.calculate(
                driveSubsystem.getRotation().getRadians(), getBumpLockAngle().getRadians());
        driveSubsystem.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            RadiansPerSecond.of(rotSpeedToDiagonal));
        break;
    }
  }

  private void setDriveSpeed(LinearVelocity speed) {
    maxDriveSpeed = speed;
  }

  private void setRotSpeed(AngularVelocity speed) {
    maxRotSpeed = speed;
  }

  public Command speedUpCommand() {
    return Commands.startEnd(
        () -> {
          setDriveSpeed(DriveConstants.fastDriveSpeed);
          setRotSpeed(DriveConstants.fastRotationSpeed);
        },
        () -> {
          setDriveSpeed(DriveConstants.defaultDriveSpeed);
          setRotSpeed(DriveConstants.defaultRotationSpeed);
        });
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
