// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class SimShotVisualizer {
  private SimShotVisualizer() {}

  // Balls per second
  private static final double bps = 6.0;
  private static double lastShotTime = 0.0;

  private static int score = 0;

  private static final double rpmToMetersPerSecond = 0.0034; // Multiplier from John's Excel sheet

  public static void launchFuelWithRateLimit() {
    if (Constants.isSim()
        && Timer.getFPGATimestamp() - lastShotTime > 1.0 / bps
        && IntakeSubsystem.getInstance().getIntakeSimulation().obtainGamePieceFromIntake()) {
      lastShotTime = Timer.getFPGATimestamp();

      Pose2d drivePose = DriveSubsystem.getInstance().getSimulatedDriveTrainPose();
      Rotation2d turretRotation = new Rotation2d(TurretSubsystem.getInstance().getPosition());

      RebuiltFuelOnFly fuelOnFly =
          new RebuiltFuelOnFly(
              // Specify the position of the chassis when the note is launched
              drivePose.getTranslation(),
              // Specify the translation of the shooter from the robot center (in the shooter’s
              // reference frame)
              TurretConstants.robotToTurret
                  .getTranslation()
                  .toTranslation2d()
                  .rotateBy(turretRotation.unaryMinus()),
              // Specify the field-relative speed of the chassis, adding it to the initial
              // velocity
              // of the projectile
              DriveSubsystem.getInstance().getFieldRelativeChassisSpeeds(),
              // The shooter facing direction is the same as the robot’s facing direction
              drivePose
                  .getRotation()
                  // Add the shooter’s rotation
                  .plus(turretRotation),
              // Initial height of the fuel
              Meters.of(TurretConstants.robotToTurret.getTranslation().getZ()),
              // Multiplier from John's Excel sheet
              Meters.per(Second)
                  .of(ShooterSubsystem.getInstance().getFlywheelRPM() * rpmToMetersPerSecond),
              // The angle at which the fuel is launched
              Radians.of(ShooterSubsystem.getInstance().getHoodAngle()));

      fuelOnFly
          // Set the target center to the Rebuilt Hub of the current alliance
          .withTargetPosition(() -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint))
          // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of hub opening)
          .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
          // Set a callback to run when the fuel hits the target
          .withHitTargetCallBack(() -> Logger.recordOutput("FieldSimulation/Score", ++score));

      fuelOnFly
          // Configure callbacks to visualize the flight trajectory of the projectile
          .withProjectileTrajectoryDisplayCallBack(
          // Callback for when the fuel will eventually hit the target (if configured)
          (pose3ds) ->
              Logger.recordOutput(
                  "FieldSimulation/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the fuel will eventually miss the target, or if no target is
          // configured
          (pose3ds) ->
              Logger.recordOutput(
                  "FieldSimulation/FuelProjectileUnsuccessfulShot",
                  pose3ds.toArray(Pose3d[]::new)));

      fuelOnFly
          // Configure the note projectile to become a NoteOnField upon touching the ground
          .enableBecomesGamePieceOnFieldAfterTouchGround();

      // Add the projectile to the simulated arena
      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }
  }
}
