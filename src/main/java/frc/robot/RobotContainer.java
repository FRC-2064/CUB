// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Cameras;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Vision vision;
  public final SuperStructure superStructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<String> yearChooser;

  // Joystick deadband for manual control detection
  private static final double JOYSTICK_DEADBAND = 0.1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(
                    "limelight-one", Cameras.LIMELIGHT_ONE.cameraToRobot, drive::getRotation),
                new VisionIOLimelight(
                    "limelight-two", Cameras.LIMELIGHT_TWO.cameraToRobot, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Initialize SuperStructure
    superStructure = new SuperStructure(drive, vision);
    // Set alignment camera (0 = first limelight, 1 = second limelight)
    superStructure.setAlignmentCamera(0);

    // Set up year chooser
    yearChooser = new SendableChooser<>();
    yearChooser.setDefaultOption("Crescendo (2024)", "Crescendo");
    yearChooser.addOption("Reefscape (2025)", "Reefscape");
    SmartDashboard.putData("Kapok Year", yearChooser);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Checks if the driver is providing manual joystick input.
   *
   * @return true if any joystick axis exceeds the deadband threshold
   */
  private boolean hasManualInput() {
    return Math.abs(controller.getLeftY()) > JOYSTICK_DEADBAND
        || Math.abs(controller.getLeftX()) > JOYSTICK_DEADBAND
        || Math.abs(controller.getRightX()) > JOYSTICK_DEADBAND;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Cancel SuperStructure alignment/aiming when driver moves joystick
    new Trigger(this::hasManualInput)
        .onTrue(
            Commands.runOnce(
                () -> {
                  SuperStructure.State currentState = superStructure.getCurrentState();
                  // Cancel alignment, aiming, or searching, but not snap-to-target or manual
                  if (currentState == SuperStructure.State.ALIGNING
                      || currentState == SuperStructure.State.AIMING
                      || currentState == SuperStructure.State.SEARCHING) {
                    superStructure.stopAndReturnToManual();
                  }
                },
                superStructure));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Y button - Toggle alignment to AprilTag 7 (stays in range, follows the tag)
    // First press: Start following and maintaining position relative to tag
    // Second press: Stop following and return to manual control
    // Auto-cancels: Yes (on joystick movement)
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (superStructure.getCurrentState() == SuperStructure.State.ALIGNING) {
                    // Already aligning, stop and return to manual
                    superStructure.stopAndReturnToManual();
                  } else {
                    // Start alignment - will continuously follow the tag
                    superStructure.startAlignment(
                        7, // Target tag ID
                        new Transform2d(
                            new Translation2d(1.0, 0.0), // 1m in front of tag
                            Rotation2d.fromDegrees(180.0))); // Face the tag
                  }
                },
                superStructure));

    // Right Bumper - Toggle aim at AprilTag 7 (rotation only, tracks tag rotation)
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (superStructure.getCurrentState() == SuperStructure.State.AIMING) {
                    // Already aiming, stop and return to manual
                    superStructure.stopAndReturnToManual();
                  } else {
                    // Start aiming - will continuously track tag rotation
                    superStructure.startAiming(7);
                  }
                },
                superStructure));

    // Left Bumper - Toggle snap-to-target mode for AprilTag 7
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superStructure.toggleSnapToTarget(7), superStructure));

    // Back button - Emergency stop SuperStructure and return to manual
    controller
        .back()
        .onTrue(Commands.runOnce(superStructure::stopAndReturnToManual, superStructure));
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
