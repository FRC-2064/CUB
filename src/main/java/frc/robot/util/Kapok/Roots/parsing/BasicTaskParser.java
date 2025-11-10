package frc.robot.util.Kapok.Roots.parsing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Kapok.Roots.core.Task;

/**
 * Parser for basic, game-agnostic task strings. Handles commands like DELAY and POSE that work
 * across all games.
 */
public class BasicTaskParser {

  /**
   * Try to parse a task string as a basic command.
   *
   * @param taskString The task string to parse
   * @return A Task if this is a basic command, null otherwise
   */
  public static Task tryParse(String taskString) {
    String[] parts = taskString.split(":");
    if (parts.length < 1) {
      return null;
    }

    String action = parts[0];

    switch (action) {
      case "DELAY":
        return parseDelay(parts);
      case "POSE":
        return parsePose(parts);
      default:
        return null; // Not a basic command
    }
  }

  /**
   * Parse a DELAY command. Format: DELAY:milliseconds Example: DELAY:1000 (wait 1 second)
   *
   * @param parts The split task string
   * @return A DELAY task
   */
  private static Task parseDelay(String[] parts) {
    if (parts.length < 2) {
      throw new IllegalArgumentException("DELAY requires format: DELAY:milliseconds");
    }

    try {
      double milliseconds = Double.parseDouble(parts[1]);
      double seconds = milliseconds / 1000.0;

      return new Task("DELAY", String.valueOf(seconds), new Pose2d(), new Pose2d(), -1);
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException(
          "Invalid delay duration: %s. Must be a number in milliseconds.".formatted(parts[1]));
    }
  }

  /**
   * Parse a POSE command. Format: POSE:x,y,theta Example: POSE:1.5,2.3,45.0 (drive to x=1.5m,
   * y=2.3m, theta=45 degrees)
   *
   * @param parts The split task string
   * @return A POSE task
   */
  private static Task parsePose(String[] parts) {
    if (parts.length < 2) {
      throw new IllegalArgumentException("POSE requires format: POSE:x,y,theta");
    }

    String[] coordinates = parts[1].split(",");
    if (coordinates.length != 3) {
      throw new IllegalArgumentException(
          "POSE requires exactly 3 coordinates (x,y,theta). Got: %s".formatted(parts[1]));
    }

    try {
      double x = Double.parseDouble(coordinates[0].trim());
      double y = Double.parseDouble(coordinates[1].trim());
      double thetaDegrees = Double.parseDouble(coordinates[2].trim());

      Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees));

      return new Task(
          "POSE", String.format("%.2f,%.2f,%.1f", x, y, thetaDegrees), targetPose, targetPose, -1);
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException(
          "Invalid coordinates: %s. All values must be numbers.".formatted(parts[1]));
    }
  }
}
