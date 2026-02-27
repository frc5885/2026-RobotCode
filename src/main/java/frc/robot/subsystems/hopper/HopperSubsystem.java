// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.kicker.KickerIO;
import frc.robot.subsystems.hopper.kicker.KickerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.kicker.KickerIOSim;
import frc.robot.subsystems.hopper.kicker.KickerIOSpark;
import frc.robot.subsystems.hopper.spindexer.SpindexerIO;
import frc.robot.subsystems.hopper.spindexer.SpindexerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.spindexer.SpindexerIOSim;
import frc.robot.subsystems.hopper.spindexer.SpindexerIOSpark;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
  private static HopperSubsystem INSTANCE = null;

  // Hopper bowl center position (robot-relative, meters)
  private static final double HOPPER_CENTER_X = 0.12;
  private static final double HOPPER_CENTER_Y = -0.045;
  private static final double HOPPER_FLOOR_Z = 0.15;

  // Ball ring layout — 6 balls per layer, 5 layers = 30 ball capacity
  private static final int BALLS_PER_RING = 6;
  private static final int MAX_RING_LAYERS = 5;
  private static final double BALL_DIAMETER = 0.075; // meters (~3 inches)
  private static final double BALL_RING_RADIUS = 0.12; // ring radius around hopper center

  // Pre-compute the fixed robot-relative position for every ball slot (slot index
  // = layer * 6 + i)
  private static final Translation3d[] BALL_SLOT_POSITIONS =
      computeBallSlotPositions(BALLS_PER_RING, MAX_RING_LAYERS);

  private static Translation3d[] computeBallSlotPositions(int ballsPerRing, int layers) {
    Translation3d[] positions = new Translation3d[ballsPerRing * layers];
    for (int layer = 0; layer < layers; layer++) {
      for (int i = 0; i < ballsPerRing; i++) {
        double angle = (2 * Math.PI / ballsPerRing) * i;
        double rx = HOPPER_CENTER_X + BALL_RING_RADIUS * Math.cos(angle);
        double ry = HOPPER_CENTER_Y + BALL_RING_RADIUS * Math.sin(angle);
        double rz = HOPPER_FLOOR_Z + layer * BALL_DIAMETER;
        positions[layer * ballsPerRing + i] = new Translation3d(rx, ry, rz);
      }
    }
    return positions;
  }

  public static HopperSubsystem getInstance() {
    if (INSTANCE == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          INSTANCE = new HopperSubsystem(new KickerIOSpark(), new SpindexerIOSpark());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          INSTANCE = new HopperSubsystem(new KickerIOSim(), new SpindexerIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          INSTANCE = new HopperSubsystem(new KickerIO() {}, new SpindexerIO() {});
          break;
      }
    }

    return INSTANCE;
  }

  private final Alert kickerDisconnectedAlert;
  private final Alert spindexerDisconnectedAlert;
  private final KickerIO kickerIO;
  private final SpindexerIO spindexerIO;
  private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();
  private final SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();

  /** Creates a new Hopper. */
  private HopperSubsystem(KickerIO kickerIO, SpindexerIO spindexerIO) {
    this.kickerIO = kickerIO;
    this.spindexerIO = spindexerIO;
    kickerDisconnectedAlert = new Alert("Kicker motor disconnected!", AlertType.kError);
    spindexerDisconnectedAlert = new Alert("Spindexer motor disconnected!", AlertType.kError);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    kickerIO.updateInputs(kickerInputs);
    spindexerIO.updateInputs(spindexerInputs);
    Logger.processInputs("Hopper/Kicker", kickerInputs);
    Logger.processInputs("Hopper/Spindexer", spindexerInputs);

    kickerDisconnectedAlert.set(!kickerInputs.motorConnected);
    spindexerDisconnectedAlert.set(!spindexerInputs.motorConnected);

    visualizationUpdate();
  }

  public void setKickerVoltage(double volts) {
    kickerIO.setMotorVoltage(volts);
  }

  public void setSpindexerVoltage(double volts) {
    spindexerIO.setMotorVoltage(volts);
  }

  private void visualizationUpdate() {
    // Log the spindexer rotation as a 3D pose for mechanism visualization
    Logger.recordOutput(
        "Mechanism3d/0-Spindexer",
        new Pose3d(
            HOPPER_CENTER_X,
            HOPPER_CENTER_Y,
            HOPPER_FLOOR_Z,
            new Rotation3d(0.0, 0.0, Units.rotationsToRadians(spindexerInputs.positionRotations))));

    // Visualize balls held inside the hopper in sim only
    if (Constants.currentMode == Constants.Mode.SIM) {
      int ballCount = IntakeSubsystem.getInstance().getSimHopperFuelCount();

      Pose3d robotPose = new Pose3d(DriveSubsystem.getInstance().getSimulatedDriveTrainPose());
      Pose3d[] ballPoses = new Pose3d[ballCount];
      Pose3d[] ballPosesRobotRelative = new Pose3d[ballCount];

      // Each ball occupies a fixed pre-computed slot (ring position + layer height).
      // Robot-relative positions never change, so balls stay anchored to the hopper.
      for (int i = 0; i < ballCount; i++) {
        Translation3d slotPosition = BALL_SLOT_POSITIONS[i % BALL_SLOT_POSITIONS.length];
        ballPosesRobotRelative[i] = new Pose3d(slotPosition, new Rotation3d());
        ballPoses[i] = robotPose.transformBy(new Transform3d(slotPosition, new Rotation3d()));
      }

      Logger.recordOutput("FieldSimulation/HopperFuel", ballPoses);
    }
  }
}
