package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * SuperStructure manages high-level robot coordination and AprilTag-based alignment.
 *
 * <p>Features:
 *
 * <ul>
 *   <li>State machine-based control
 *   <li>Vision-based alignment to AprilTags with configurable offset
 *   <li>Hold-to-aim mode for precise targeting
 *   <li>Snap-to-target toggle that auto-rotates when tag is visible
 *   <li>Gyro-based tag recovery when vision is lost
 *   <li>Dedicated camera support for alignment
 * </ul>
 */
public class SuperStructure extends SubsystemBase {
  /** SuperStructure operating states */
  public enum State {
    /** Manual control - no automated behavior */
    MANUAL,

    /** Driving to and aligning with an AprilTag */
    ALIGNING,

    /** Holding aim at a tag (rotation only) */
    AIMING,

    /** Snap-to-target mode - auto-rotates when tag visible */
    SNAP_TO_TARGET,

    /** Searching for lost tag using gyro predictions */
    SEARCHING,

    /** Robot stopped and holding position */
    STOPPED
  }

  // Subsystem dependencies
  private final Drive drive;
  private final Vision vision;

  // Dedicated camera for alignment (index into vision inputs array)
  private int alignmentCameraIndex = 0;

  // State machine
  private State currentState = State.MANUAL;
  private State wantedState = State.MANUAL;

  // Target tracking
  private int targetTagId = -1;
  private Transform2d targetOffset = new Transform2d();
  private Pose2d lastKnownTagPose = null;
  private Pose2d desiredRobotPose = null;
  private double lastTagSeenTime = 0.0;

  // PID Controllers for alignment
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  // PID Controller for aiming (rotation only)
  private final ProfiledPIDController aimController;

  /** Creates a new SuperStructure */
  public SuperStructure(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;

    // Initialize alignment controllers
    xController =
        new ProfiledPIDController(
            Cub.SuperStructureAlignment.ALIGNMENT_LINEAR_KP,
            Cub.SuperStructureAlignment.ALIGNMENT_LINEAR_KI,
            Cub.SuperStructureAlignment.ALIGNMENT_LINEAR_KD,
            new TrapezoidProfile.Constraints(
                Cub.SuperStructureAlignment.MAX_ALIGNMENT_VELOCITY.in(MetersPerSecond),
                Cub.SuperStructureAlignment.MAX_ALIGNMENT_ACCELERATION.in(
                    MetersPerSecondPerSecond)));

    yController =
        new ProfiledPIDController(
            Cub.SuperStructureAlignment.ALIGNMENT_LINEAR_KP,
            Cub.SuperStructureAlignment.ALIGNMENT_LINEAR_KI,
            Cub.SuperStructureAlignment.ALIGNMENT_LINEAR_KD,
            new TrapezoidProfile.Constraints(
                Cub.SuperStructureAlignment.MAX_ALIGNMENT_VELOCITY.in(MetersPerSecond),
                Cub.SuperStructureAlignment.MAX_ALIGNMENT_ACCELERATION.in(
                    MetersPerSecondPerSecond)));

    thetaController =
        new ProfiledPIDController(
            Cub.SuperStructureAlignment.ALIGNMENT_ANGULAR_KP,
            Cub.SuperStructureAlignment.ALIGNMENT_ANGULAR_KI,
            Cub.SuperStructureAlignment.ALIGNMENT_ANGULAR_KD,
            new TrapezoidProfile.Constraints(
                Cub.SuperStructureAlignment.MAX_ALIGNMENT_ANGULAR_VELOCITY.in(RadiansPerSecond),
                Cub.SuperStructureAlignment.MAX_ALIGNMENT_ANGULAR_ACCELERATION.in(
                    RadiansPerSecond.per(Second))));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize aiming controller
    aimController =
        new ProfiledPIDController(
            Cub.SuperStructureAlignment.AIM_ANGULAR_KP,
            Cub.SuperStructureAlignment.AIM_ANGULAR_KI,
            Cub.SuperStructureAlignment.AIM_ANGULAR_KD,
            new TrapezoidProfile.Constraints(
                Cub.SuperStructureAlignment.MAX_AIM_ANGULAR_VELOCITY.in(RadiansPerSecond),
                Cub.SuperStructureAlignment.MAX_AIM_ANGULAR_ACCELERATION.in(
                    RadiansPerSecond.per(Second))));
    aimController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Sets the camera index to use for alignment.
   *
   * @param cameraIndex Index of the camera in the vision subsystem (0 or 1)
   */
  public void setAlignmentCamera(int cameraIndex) {
    this.alignmentCameraIndex = cameraIndex;
  }

  /**
   * Gets the current camera index used for alignment.
   *
   * @return The camera index
   */
  public int getAlignmentCamera() {
    return alignmentCameraIndex;
  }

  /**
   * Requests a state change.
   *
   * @param newState The desired state
   */
  public void setWantedState(State newState) {
    this.wantedState = newState;
  }

  /**
   * Gets the current state.
   *
   * @return The current state
   */
  public State getCurrentState() {
    return currentState;
  }

  /**
   * Starts alignment to an AprilTag with a specified offset.
   *
   * @param tagId The AprilTag ID to align to
   * @param offset The desired offset from the tag (Translation2d and Rotation2d)
   */
  public void startAlignment(int tagId, Transform2d offset) {
    this.targetTagId = tagId;
    this.targetOffset = offset;
    setWantedState(State.ALIGNING);
  }

  /**
   * Starts aiming at an AprilTag (rotation only, no translation).
   *
   * @param tagId The AprilTag ID to aim at
   */
  public void startAiming(int tagId) {
    this.targetTagId = tagId;
    setWantedState(State.AIMING);
  }

  /**
   * Toggles snap-to-target mode. When enabled, the robot will automatically rotate to face the
   * target tag whenever it's visible.
   */
  public void toggleSnapToTarget(int tagId) {
    this.targetTagId = tagId;
    if (currentState == State.SNAP_TO_TARGET) {
      setWantedState(State.MANUAL);
    } else {
      setWantedState(State.SNAP_TO_TARGET);
    }
  }

  /** Stops all SuperStructure actions and returns to manual control. */
  public void stopAndReturnToManual() {
    setWantedState(State.MANUAL);
  }

  /**
   * Checks if the robot is aligned to the target.
   *
   * @return true if aligned within tolerances
   */
  public boolean isAligned() {
    if (desiredRobotPose == null) return false;

    Pose2d currentPose = drive.getPose();
    double positionError =
        currentPose.getTranslation().getDistance(desiredRobotPose.getTranslation());
    double rotationError =
        Math.abs(currentPose.getRotation().minus(desiredRobotPose.getRotation()).getRadians());

    return positionError < Cub.SuperStructureAlignment.POSITION_TOLERANCE.in(Meters)
        && rotationError < Cub.SuperStructureAlignment.ROTATION_TOLERANCE.in(Radians);
  }

  /**
   * Checks if the robot is aimed at the target.
   *
   * @return true if aimed within tolerance
   */
  public boolean isAimed() {
    if (lastKnownTagPose == null) return false;

    Pose2d currentPose = drive.getPose();
    Rotation2d angleToTag =
        lastKnownTagPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

    double rotationError = Math.abs(currentPose.getRotation().minus(angleToTag).getRadians());

    return rotationError < Cub.SuperStructureAlignment.AIM_TOLERANCE.in(Radians);
  }

  @Override
  public void periodic() {
    // Handle state transitions
    handleStateTransitions();

    // Execute state-specific logic
    switch (currentState) {
      case MANUAL:
        // No automated control
        break;

      case ALIGNING:
        executeAligning();
        break;

      case AIMING:
        executeAiming();
        break;

      case SNAP_TO_TARGET:
        executeSnapToTarget();
        break;

      case SEARCHING:
        executeSearching();
        break;

      case STOPPED:
        drive.stop();
        break;
    }

    // Logging
    Logger.recordOutput("SuperStructure/CurrentState", currentState.toString());
    Logger.recordOutput("SuperStructure/WantedState", wantedState.toString());
    Logger.recordOutput("SuperStructure/TargetTagId", targetTagId);
    Logger.recordOutput("SuperStructure/TagVisible", isTagVisible());
    Logger.recordOutput("SuperStructure/IsAligned", isAligned());
    Logger.recordOutput("SuperStructure/IsAimed", isAimed());
    if (desiredRobotPose != null) {
      Logger.recordOutput("SuperStructure/DesiredPose", desiredRobotPose);
    }
    if (lastKnownTagPose != null) {
      Logger.recordOutput("SuperStructure/LastKnownTagPose", lastKnownTagPose);
    }
  }

  /** Handles state transitions based on conditions. */
  private void handleStateTransitions() {
    // Check if we want to change states
    if (wantedState != currentState) {
      // Exit current state
      exitState(currentState);

      // Enter new state
      currentState = wantedState;
      enterState(currentState);
    }

    // Auto-transition to SEARCHING if tag is lost during ALIGNING or AIMING
    if ((currentState == State.ALIGNING || currentState == State.AIMING) && isTagLost()) {
      wantedState = State.SEARCHING;
    }

    // Auto-transition back to previous state if tag is found during SEARCHING
    if (currentState == State.SEARCHING && isTagVisible()) {
      // Return to the state we were in before searching
      wantedState = State.ALIGNING; // Default to aligning
    }
  }

  /** Called when entering a new state. */
  private void enterState(State state) {
    switch (state) {
      case ALIGNING:
        // Reset PID controllers
        xController.reset(drive.getPose().getX());
        yController.reset(drive.getPose().getY());
        thetaController.reset(drive.getPose().getRotation().getRadians());
        break;

      case AIMING:
        // Reset aim controller
        aimController.reset(drive.getPose().getRotation().getRadians());
        break;

      case SNAP_TO_TARGET:
        // Reset aim controller for snap mode
        aimController.reset(drive.getPose().getRotation().getRadians());
        break;

      case SEARCHING:
        // Start searching from current rotation
        break;

      case MANUAL:
      case STOPPED:
      default:
        break;
    }
  }

  /** Called when exiting a state. */
  private void exitState(State state) {
    // Cleanup if needed
  }

  /** Executes the ALIGNING state - drives to and aligns with the target tag. */
  private void executeAligning() {
    Optional<Pose2d> tagPoseOpt = getTagPoseInField(targetTagId);

    if (tagPoseOpt.isEmpty()) {
      // Tag not visible, handled by state transition to SEARCHING
      return;
    }

    // Update last known tag pose and time
    lastKnownTagPose = tagPoseOpt.get();
    lastTagSeenTime = Timer.getFPGATimestamp();

    // Calculate desired robot pose based on tag pose and offset
    desiredRobotPose = lastKnownTagPose.transformBy(targetOffset);

    // Get current pose
    Pose2d currentPose = drive.getPose();

    // Calculate PID outputs for field-relative speeds
    double vx = xController.calculate(currentPose.getX(), desiredRobotPose.getX());
    double vy = yController.calculate(currentPose.getY(), desiredRobotPose.getY());
    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredRobotPose.getRotation().getRadians());

    // Create field-relative chassis speeds
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);

    // Convert to robot-relative speeds and command the drive
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());

    drive.runVelocity(robotRelativeSpeeds);
  }

  /** Executes the AIMING state - rotates to aim at the target tag. */
  private void executeAiming() {
    Optional<Pose2d> tagPoseOpt = getTagPoseInField(targetTagId);

    if (tagPoseOpt.isEmpty()) {
      // Tag not visible, handled by state transition to SEARCHING
      return;
    }

    // Update last known tag pose and time
    lastKnownTagPose = tagPoseOpt.get();
    lastTagSeenTime = Timer.getFPGATimestamp();

    // Calculate angle to tag
    Pose2d currentPose = drive.getPose();
    Rotation2d angleToTag =
        lastKnownTagPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

    // Calculate rotation speed
    double omega =
        aimController.calculate(currentPose.getRotation().getRadians(), angleToTag.getRadians());

    // Command rotation only (no translation)
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
  }

  /**
   * Executes the SNAP_TO_TARGET state - allows manual drive with auto-rotation to tag. This would
   * typically be combined with joystick input in a command.
   */
  private void executeSnapToTarget() {
    Optional<Pose2d> tagPoseOpt = getTagPoseInField(targetTagId);

    if (tagPoseOpt.isEmpty()) {
      // Tag not visible - maintain current heading but allow manual drive
      // In a real implementation, this would be handled by the command using this subsystem
      return;
    }

    // Update last known tag pose and time
    lastKnownTagPose = tagPoseOpt.get();
    lastTagSeenTime = Timer.getFPGATimestamp();

    // Calculate angle to tag
    Pose2d currentPose = drive.getPose();
    Rotation2d angleToTag =
        lastKnownTagPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

    // Calculate rotation speed (translation would come from joystick input)
    double omega =
        aimController.calculate(currentPose.getRotation().getRadians(), angleToTag.getRadians());

    // For now, just rotate (command integration would add translation)
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
  }

  /** Executes the SEARCHING state - slowly rotates to search for lost tag. */
  private void executeSearching() {
    // Check if we found the tag
    if (isTagVisible()) {
      // State transition will handle returning to previous state
      return;
    }

    // Use gyro to predict where the tag should be based on last known position
    if (lastKnownTagPose != null) {
      Pose2d currentPose = drive.getPose();
      Rotation2d expectedAngleToTag =
          lastKnownTagPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

      // Calculate rotation to expected tag position
      double rotationError = expectedAngleToTag.minus(currentPose.getRotation()).getRadians();

      // Slowly rotate towards expected position
      double omega =
          MathUtil.clamp(
              rotationError * 2.0, // Simple P controller
              -Cub.SuperStructureAlignment.SEARCH_ROTATION_SPEED.in(RadiansPerSecond),
              Cub.SuperStructureAlignment.SEARCH_ROTATION_SPEED.in(RadiansPerSecond));

      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
    } else {
      // No last known position, do a slow 360 search
      drive.runVelocity(
          new ChassisSpeeds(
              0.0, 0.0, Cub.SuperStructureAlignment.SEARCH_ROTATION_SPEED.in(RadiansPerSecond)));
    }
  }

  /**
   * Gets the field pose of a specific AprilTag.
   *
   * @param tagId The tag ID
   * @return Optional containing the tag's field pose if found
   */
  private Optional<Pose2d> getTagPoseInField(int tagId) {
    // Get tag pose from AprilTag field layout
    Optional<Pose3d> tagPose3dOpt = VisionConstants.aprilTagLayout.getTagPose(tagId);

    if (tagPose3dOpt.isEmpty()) {
      return Optional.empty();
    }

    // Verify we can actually see the tag with the alignment camera
    if (!isTagVisibleInCamera(tagId, alignmentCameraIndex)) {
      return Optional.empty();
    }

    // Convert 3D pose to 2D
    return Optional.of(tagPose3dOpt.get().toPose2d());
  }

  /**
   * Checks if a specific tag is visible in a specific camera.
   *
   * @param tagId The tag ID to check
   * @param cameraIndex The camera index
   * @return true if the tag is visible
   */
  private boolean isTagVisibleInCamera(int tagId, int cameraIndex) {
    var inputs = vision.getInputs();
    if (cameraIndex >= inputs.length) {
      return false;
    }

    var cameraInputs = inputs[cameraIndex];

    // Check if any pose observations contain this tag
    for (PoseObservation observation : cameraInputs.poseObservations) {
      if (Arrays.stream(observation.tagIds()).anyMatch(id -> id == tagId)) {
        return true;
      }
    }

    // Also check tag IDs array
    return Arrays.stream(cameraInputs.tagIds).anyMatch(id -> id == tagId);
  }

  /**
   * Checks if the target tag is currently visible.
   *
   * @return true if visible
   */
  private boolean isTagVisible() {
    return targetTagId >= 0 && isTagVisibleInCamera(targetTagId, alignmentCameraIndex);
  }

  /**
   * Checks if the tag has been lost (not seen for timeout period).
   *
   * @return true if lost
   */
  private boolean isTagLost() {
    return !isTagVisible()
        && (Timer.getFPGATimestamp() - lastTagSeenTime)
            > Cub.SuperStructureAlignment.TAG_LOST_TIMEOUT.in(Seconds);
  }

  /**
   * Creates a command that drives to and aligns with an AprilTag. This method would typically be
   * called from RobotContainer.
   *
   * @param tagId The tag ID to align to
   * @param offset The desired offset from the tag
   * @return A command that aligns to the tag
   */
  public void align(int tagId, Transform2d offset) {
    startAlignment(tagId, offset);
  }

  /**
   * Creates a command that aims at an AprilTag (rotation only).
   *
   * @param tagId The tag ID to aim at
   */
  public void aim(int tagId) {
    startAiming(tagId);
  }
}
