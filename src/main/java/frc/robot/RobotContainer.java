// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AssistedDriveCommand;
import frc.robot.commands.DefaultCommands;
import frc.robot.commands.DriveToClimbPoseSequentialCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.SetBrakeModeCommand;
import frc.robot.commands.ShiftChangeRumbleLEDCommand;
import frc.robot.commands.SysIDCommands;
import frc.robot.commands.autonomous.PreSpinFlywheelCommand;
import frc.robot.commands.autonomous.ShootUntilHopperEmptyCommand;
import frc.robot.commands.autonomous.StopDrivingCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeControlCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.shooting.ShootCommandGroup;
import frc.robot.commands.shooting.TurretCommands;
import frc.robot.controllers.OperatorPanel;
import frc.robot.subsystems.leds.LEDConstants.LEDState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final OperatorPanel operatorPanel = new OperatorPanel(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init all subsystems
    SubsystemFactory.initAllSubsystems();

    // Init all named commands
    // must be before the set up auto routines
    NamedCommands.registerCommand("Intake", new IntakeCommand());
    NamedCommands.registerCommand("Shoot", new ShootUntilHopperEmptyCommand());
    NamedCommands.registerCommand(
        "ShootWithAgitate", new ShootUntilHopperEmptyCommand().withAgitation(1.0));
    NamedCommands.registerCommand("RetractIntake", new RetractIntakeCommand());
    NamedCommands.registerCommand("PreSpinFlywheel", new PreSpinFlywheelCommand());
    NamedCommands.registerCommand("Stop", new StopDrivingCommand());
    NamedCommands.registerCommand(
        "DriveToPose", new DriveToPoseCommand(() -> new Pose2d(2.5, 5, new Rotation2d())));
    NamedCommands.registerCommand(
        "DriveToClimbPoseSequentialCommand", new DriveToClimbPoseSequentialCommand());

    SmartDashboard.putBoolean("ShootPreload", false);
    NamedCommands.registerCommand(
        "ConditionalShootPreload", new ShootUntilHopperEmptyCommand().conditionalShootPreload());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // SysIDCommands.addDriveSysIdToAutoChooser(autoChooser);
    SysIDCommands.addTurretSysIdToAutoChooser(autoChooser);
    // SysIDCommands.addHoodSysIdToAutoChooser(autoChooser);
    // SysIDCommands.addFlywheelSysIdToAutoChooser(autoChooser);
    // SysIDCommands.addExtensionSysIdToAutoChooser(autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Shift change pulse consumer
    // This runs every time HubShiftUtil fires a pulse
    HubShiftUtil.setShiftChangeConsumer(
        (pulseDuration) -> {
          new ShiftChangeRumbleLEDCommand(controller, pulseDuration).schedule();
        });
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    DefaultCommands.setDefaultDriveCommand(new AssistedDriveCommand(controller));
    DefaultCommands.setDefaultTurretCommand(
        TurretCommands.trackTargetInTeleopAndStraightForwardInTest());

    DefaultCommands.setDefaultIntakeCommand(new IntakeControlCommand(controller));

    controller.rightTrigger(0.1).whileTrue(new ShootCommandGroup());

    // controller
    //     .povLeft()
    //     .whileTrue(new DriveToPoseCommand(() -> new Pose2d(1.5, 5, new Rotation2d())));
    // controller.povRight().whileTrue(new DriveToClimbPoseSequentialCommand());

    // todo convert to state machine
    controller.b().whileTrue(new OuttakeCommand());

    // Operator Switches
    // operatorPanel
    //     .getBrakeModeSwitch()
    controller
        .start()
        .onTrue(new SetBrakeModeCommand(false).ignoringDisable(true))
        .onFalse(new SetBrakeModeCommand(true).ignoringDisable(true));

    controller
        .povLeft()
        .whileTrue(
            LEDSubsystem.getInstance().applyState(LEDState.TEST_PATTERN).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
