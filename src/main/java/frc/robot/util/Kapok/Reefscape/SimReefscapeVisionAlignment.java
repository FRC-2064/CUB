package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Cub;
import frc.robot.util.Kapok.Roots.vision.VisionAlignmentHelper;

/**
 * Simulation compatible vision alignment that uses odometry only alignment.
 *
 * <p>Uses simple PID controllers (not profiled) for more predictable sim behavior.
 */
public class SimReefscapeVisionAlignment extends VisionAlignmentHelper {
  private final RobotContainer robot;

  private final PIDController xController =
      new PIDController(
          Cub.VisionAlignment.SIM_LINEAR_KP,
          Cub.VisionAlignment.SIM_LINEAR_KI,
          Cub.VisionAlignment.SIM_LINEAR_KD);
  private final PIDController yController =
      new PIDController(
          Cub.VisionAlignment.SIM_LINEAR_KP,
          Cub.VisionAlignment.SIM_LINEAR_KI,
          Cub.VisionAlignment.SIM_LINEAR_KD);
  private final PIDController thetaController =
      new PIDController(
          Cub.VisionAlignment.SIM_ANGULAR_KP,
          Cub.VisionAlignment.SIM_ANGULAR_KI,
          Cub.VisionAlignment.SIM_ANGULAR_KD);

  public SimReefscapeVisionAlignment(RobotContainer robot) {
    this.robot = robot;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void setTarget(Pose2d targetPose) {
    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    thetaController.setSetpoint(targetPose.getRotation().getRadians());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Kapok/VisionAlignment/Sim/TargetSet", targetPose);
  }

  @Override
  public AlignmentResult alignToTarget(int targetTagID, Pose2d targetPose) {
    // Get the current robot pose from odometry
    Pose2d currentRobotPose = robot.drive.getPose();

    // Use the target pose
    Pose2d desiredRobotPose = targetPose;

    // Calculate errors
    double xError = desiredRobotPose.getX() - currentRobotPose.getX();
    double yError = desiredRobotPose.getY() - currentRobotPose.getY();
    double thetaError =
        desiredRobotPose.getRotation().getRadians() - currentRobotPose.getRotation().getRadians();

    // Calculate PID outputs for field-relative speeds
    double vx = xController.calculate(currentRobotPose.getX());
    double vy = yController.calculate(currentRobotPose.getY());
    double omega = thetaController.calculate(currentRobotPose.getRotation().getRadians());

    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Sim/XError", xError);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Sim/YError", yError);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Sim/ThetaError", thetaError);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Sim/VX_Command", vx);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Sim/VY_Command", vy);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Sim/Omega_Command", omega);

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

    // Always report tag as visible in simulation
    return new AlignmentResult(isAligned, currentRobotPose, true, robotRelativeSpeeds);
  }
}
