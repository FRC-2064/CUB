package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.util.Kapok.Roots.VisionAlignmentHelper;
import java.util.Arrays;
import java.util.Optional;

public class ReefscapeVisionAlignment extends VisionAlignmentHelper {
  private final RobotContainer robot;

  private static final double LINEAR_KP = 2.5;
  private static final double LINEAR_KI = 0.0;
  private static final double LINEAR_KD = 0.0;

  private static final double ANGULAR_KP = 2.5;
  private static final double ANGULAR_KI = 0.0;
  private static final double ANGULAR_KD = 0.0;

  private static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
  private static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2.0;
  private static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2.0;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          LINEAR_KP,
          LINEAR_KI,
          LINEAR_KD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          LINEAR_KP,
          LINEAR_KI,
          LINEAR_KD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          ANGULAR_KP,
          ANGULAR_KI,
          ANGULAR_KD,
          new TrapezoidProfile.Constraints(
              MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  public ReefscapeVisionAlignment(RobotContainer robot) {
    this.robot = robot;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public AlignmentResult alignToTarget(int targetTagID, Pose2d targetPose) {
    Optional<PoseObservation> observation = getObservation(targetTagID);

    if (observation.isEmpty()) {
      return new AlignmentResult(false, new Pose2d(), false, new ChassisSpeeds());
    }

    Optional<Pose3d> tagPose3dOpt =
        frc.robot.subsystems.vision.VisionConstants.aprilTagLayout.getTagPose(targetTagID);

    if (tagPose3dOpt.isEmpty()) {
      return new AlignmentResult(false, new Pose2d(), false, new ChassisSpeeds());
    }
    Pose3d tagPose3d = tagPose3dOpt.get();

    // Calculate the desired robot pose in the field frame
    Pose2d desiredRobotPose =
        tagPose3d
            .transformBy(
                new Transform3d(
                    targetPose.getTranslation(),
                    new Rotation3d(0, 0, targetPose.getRotation().getRadians())))
            .toPose2d();

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
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds, currentRobotPose.getRotation());

    // Check if aligned
    boolean isAligned =
        currentRobotPose.getTranslation().getDistance(desiredRobotPose.getTranslation()) < 0.1
            && Math.abs(
                    currentRobotPose.getRotation().minus(desiredRobotPose.getRotation()).getRadians())
                < Units.degreesToRadians(2.0);

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
