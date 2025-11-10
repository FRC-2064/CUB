package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Cub;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.util.Kapok.Roots.vision.VisionAlignmentHelper;
import java.util.Arrays;
import java.util.Optional;

public class ReefscapeVisionAlignment extends VisionAlignmentHelper {
  private final RobotContainer robot;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          Cub.VisionAlignment.LINEAR_KP,
          Cub.VisionAlignment.LINEAR_KI,
          Cub.VisionAlignment.LINEAR_KD,
          new TrapezoidProfile.Constraints(
              Cub.VisionAlignment.MAX_VELOCITY_MPS, Cub.VisionAlignment.MAX_ACCELERATION_MPSS));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          Cub.VisionAlignment.LINEAR_KP,
          Cub.VisionAlignment.LINEAR_KI,
          Cub.VisionAlignment.LINEAR_KD,
          new TrapezoidProfile.Constraints(
              Cub.VisionAlignment.MAX_VELOCITY_MPS, Cub.VisionAlignment.MAX_ACCELERATION_MPSS));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          Cub.VisionAlignment.ANGULAR_KP,
          Cub.VisionAlignment.ANGULAR_KI,
          Cub.VisionAlignment.ANGULAR_KD,
          new TrapezoidProfile.Constraints(
              Cub.VisionAlignment.MAX_ANGULAR_VELOCITY_RPS,
              Cub.VisionAlignment.MAX_ANGULAR_ACCELERATION_RPSS));

  public ReefscapeVisionAlignment(RobotContainer robot) {
    this.robot = robot;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void setTarget(Pose2d targetPose) {
    // Set the goal for each controller so they're pre-configured
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());
  }

  @Override
  public AlignmentResult alignToTarget(int targetTagID, Pose2d targetPose) {
    Optional<PoseObservation> observation = getObservation(targetTagID);

    if (observation.isEmpty()) {
      return new AlignmentResult(false, new Pose2d(), false, new ChassisSpeeds());
    }

    // The target pose is already in field coordinates use it directly
    Pose2d desiredRobotPose = targetPose;

    // Get the current robot pose from odometry
    Pose2d currentRobotPose = robot.drive.getPose();

    // Calculate PID outputs for field-relative speeds
    double vx = xController.calculate(currentRobotPose.getX(), desiredRobotPose.getX());
    double vy = yController.calculate(currentRobotPose.getY(), desiredRobotPose.getY());
    double omega =
        thetaController.calculate(
            currentRobotPose.getRotation().getRadians(),
            desiredRobotPose.getRotation().getRadians());

    // Create field-relative chassis speeds
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);

    // Convert to robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentRobotPose.getRotation());

    // Check if aligned
    boolean isAligned =
        currentRobotPose.getTranslation().getDistance(desiredRobotPose.getTranslation())
                < Cub.VisionAlignment.POSITION_TOLERANCE_METERS
            && Math.abs(
                    currentRobotPose
                        .getRotation()
                        .minus(desiredRobotPose.getRotation())
                        .getRadians())
                < Cub.VisionAlignment.ROTATION_TOLERANCE_RADIANS;

    return new AlignmentResult(isAligned, currentRobotPose, true, robotRelativeSpeeds);
  }

  private Optional<PoseObservation> getObservation(int tagId) {
    for (var cameraInputs : robot.vision.getInputs()) {
      for (var observation : cameraInputs.poseObservations) {
        if (Arrays.stream(observation.tagIds()).anyMatch(id -> id == tagId)) {
          return Optional.of(observation);
        }
      }
    }
    return Optional.empty();
  }
}
