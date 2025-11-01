package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Kapok.Roots.Task;
import java.util.HashMap;
import java.util.Map;

public class ReefscapeTaskParser {
  private static final Map<String, Pose2d> REEF_LOCATIONS = new HashMap<>();
  private static final Map<String, Integer> REEF_TAG_IDS = new HashMap<>();
  private static final Map<String, Pose2d> FEEDER_LOCATIONS = new HashMap<>();

  static {
    REEF_LOCATIONS.put("F3", new Pose2d());
    REEF_LOCATIONS.put("F2", new Pose2d());
    REEF_LOCATIONS.put("E3", new Pose2d());
    REEF_LOCATIONS.put("F2", new Pose2d());

    FEEDER_LOCATIONS.put("FEEDER_L", new Pose2d());
    FEEDER_LOCATIONS.put("FEEDER_R", new Pose2d());
  }

  public static Task parseTask(String taskString) {
    String[] parts = taskString.split(":");
    if (parts.length != 2) {
      throw new IllegalArgumentException("Invalid task format: %s".formatted(taskString));
    }

    String action = parts[0];
    String location = parts[1];

    switch (action) {
      case "PICKUP":
        return createPickupTask(location);
      case "SCORE":
        return createScoreTask(location);
      default:
        throw new IllegalArgumentException("Unknown action: %s".formatted(action));
    }
  }

  private static Task createPickupTask(String location) {
    Pose2d targetPose = FEEDER_LOCATIONS.get(location);
    if (targetPose == null) {
      throw new IllegalArgumentException("Unknown feeder location: %s".formatted(location));
    }

    Pose2d approachPose = calculateApproachPose(targetPose, 0.8);

    int tagID;
    switch (location) {
      case "FEEDER_L":
        tagID = 13;
        break;
      case "FEEDER_R":
        tagID = 12;
        break;
      default:
        tagID = -1;
        break;
    }

    return new Task("PICKUP", location, approachPose, targetPose, tagID);
  }

  private static Task createScoreTask(String location) {
    Pose2d targetPose = REEF_LOCATIONS.get(location);
    Integer tagID = REEF_TAG_IDS.get(location);

    if (targetPose == null || tagID == null) {
      throw new IllegalArgumentException("Uknown reef location: %s".formatted(location));
    }

    Pose2d approachPose = calculateApproachPose(targetPose, 0.6);

    return new Task("SCORE", location, approachPose, targetPose, tagID);
  }

  private static Pose2d calculateApproachPose(Pose2d target, double distanceMeters) {
    double dx = Math.cos(target.getRotation().getRadians()) * -distanceMeters;
    double dy = Math.sin(target.getRotation().getRadians()) * -distanceMeters;
    return new Pose2d(target.getX() + dx, target.getY() + dy, target.getRotation());
  }
}
