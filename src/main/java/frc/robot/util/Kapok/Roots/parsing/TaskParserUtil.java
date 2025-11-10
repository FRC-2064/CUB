package frc.robot.util.Kapok.Roots.parsing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;

/**
 * Utility methods for parsing and processing task strings. These helpers are game-agnostic and can
 * be used across all years.
 */
public class TaskParserUtil {

  /**
   * Calculate an approach pose offset from a target pose.
   *
   * <p>The approach pose is positioned directly in front of or behind the target, based on the
   * target's rotation. This is useful for pathfinding to a position before final alignment.
   *
   * @param targetPose The final target pose
   * @param offsetDistance Distance to offset from the target (meters, positive = in front)
   * @param useInverseHeading If true, approach from the opposite direction (180 degrees rotated)
   * @return The approach pose
   */
  public static Pose2d calculateApproachPose(
      Pose2d targetPose, double offsetDistance, boolean useInverseHeading) {
    double angle = targetPose.getRotation().getRadians();
    if (useInverseHeading) {
      angle += Math.PI;
    }

    // Calculate offset in direction of heading (negative because we want to be in front)
    double dx = Math.cos(angle) * -offsetDistance;
    double dy = Math.sin(angle) * -offsetDistance;

    return new Pose2d(targetPose.getX() + dx, targetPose.getY() + dy, targetPose.getRotation());
  }

  /**
   * Calculate an approach pose offset from a target pose (simple version).
   *
   * <p>Uses normal heading direction (not inverse).
   *
   * @param targetPose The final target pose
   * @param offsetDistance Distance to offset from the target (meters)
   * @return The approach pose
   */
  public static Pose2d calculateApproachPose(Pose2d targetPose, double offsetDistance) {
    return calculateApproachPose(targetPose, offsetDistance, false);
  }

  /**
   * Parse a delay duration from a task string.
   *
   * <p>Format: "DELAY:milliseconds"
   *
   * <p>Example: "DELAY:1500" returns 1500
   *
   * @param taskString The task string to parse
   * @return The delay in milliseconds, or empty if not a delay task
   */
  public static Optional<Long> parseDelayMillis(String taskString) {
    if (!taskString.startsWith("DELAY:")) {
      return Optional.empty();
    }

    String[] parts = taskString.split(":");
    if (parts.length < 2) {
      return Optional.empty();
    }

    try {
      return Optional.of(Long.parseLong(parts[1]));
    } catch (NumberFormatException e) {
      return Optional.empty();
    }
  }

  /**
   * Parse a delay duration in seconds from a task string.
   *
   * @param taskString The task string to parse
   * @return The delay in seconds, or empty if not a delay task
   */
  public static Optional<Double> parseDelaySeconds(String taskString) {
    return parseDelayMillis(taskString).map(millis -> millis / 1000.0);
  }

  /**
   * Parse a pose from a task string.
   *
   * <p>Format: "POSE:x,y,rotation_degrees"
   *
   * <p>Example: "POSE:1.5,2.0,90" returns Pose2d(1.5, 2.0, 90 degrees)
   *
   * @param taskString The task string to parse
   * @return The pose, or empty if not a pose task or invalid format
   */
  public static Optional<Pose2d> parsePose(String taskString) {
    if (!taskString.startsWith("POSE:")) {
      return Optional.empty();
    }

    String[] parts = taskString.split(":");
    if (parts.length < 2) {
      return Optional.empty();
    }

    String[] coordinates = parts[1].split(",");
    if (coordinates.length != 3) {
      return Optional.empty();
    }

    try {
      double x = Double.parseDouble(coordinates[0].trim());
      double y = Double.parseDouble(coordinates[1].trim());
      double thetaDegrees = Double.parseDouble(coordinates[2].trim());

      return Optional.of(new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees)));
    } catch (NumberFormatException e) {
      return Optional.empty();
    }
  }

  /**
   * Check if a task string represents a dynamic/automatic task (e.g., "SCORE:AUTO", "SHOOT:AUTO").
   *
   * @param taskString The task string to check
   * @return True if this is an AUTO task
   */
  public static boolean isAutoTask(String taskString) {
    String[] parts = taskString.split(":");
    if (parts.length < 2) {
      return false;
    }
    return parts[1].equalsIgnoreCase("AUTO");
  }

  /**
   * Split a task string into action and location components.
   *
   * <p>Example: "SCORE:E2" returns ["SCORE", "E2"]
   *
   * @param taskString The task string to split
   * @return Array of [action, location], or empty array if invalid format
   */
  public static String[] splitTaskString(String taskString) {
    String[] parts = taskString.split(":");
    if (parts.length < 2) {
      return new String[0];
    }
    return parts;
  }

  /**
   * Get the action from a task string.
   *
   * <p>Example: "SCORE:E2" returns "SCORE"
   *
   * @param taskString The task string
   * @return The action, or empty if invalid format
   */
  public static Optional<String> getAction(String taskString) {
    String[] parts = splitTaskString(taskString);
    if (parts.length == 0) {
      return Optional.empty();
    }
    return Optional.of(parts[0]);
  }

  /**
   * Get the location from a task string.
   *
   * <p>Example: "SCORE:E2" returns "E2"
   *
   * @param taskString The task string
   * @return The location, or empty if invalid format
   */
  public static Optional<String> getLocation(String taskString) {
    String[] parts = splitTaskString(taskString);
    if (parts.length < 2) {
      return Optional.empty();
    }
    return Optional.of(parts[1]);
  }
}
