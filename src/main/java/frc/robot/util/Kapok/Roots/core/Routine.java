package frc.robot.util.Kapok.Roots.core;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.File;
import java.io.IOException;
import java.util.List;

/**
 * Autonomous routine loaded from JSON.
 *
 * <p>JSON format:
 *
 * <pre>{@code
 * {
 *   "name": "Right Side 4 Piece",
 *   "startingPose": [x_meters, y_meters, rotation_degrees],
 *   "tasks": [
 *     "PICKUP:FEEDER_R",
 *     "SCORE:AUTO",
 *     "PICKUP:FEEDER_R",
 *     "SCORE:E3"
 *   ]
 * }
 * }</pre>
 *
 * <p>Usage:
 *
 * <pre>{@code
 * File jsonFile = new File("src/main/deploy/Kapok/MyAuto.json");
 * Routine routine = Routine.loadFromJson(jsonFile);
 *
 * Pose2d startPose = routine.getStartingPose();
 * List<String> taskStrings = routine.getTasks();
 * }</pre>
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public class Routine {
  @JsonProperty("name")
  private String name;

  @JsonProperty("startingPose")
  private double[] startingPose; // [x, y, rotation_degrees]

  @JsonProperty("tasks")
  private List<String> tasks;

  /** Get the routine name. */
  public String getName() {
    return name;
  }

  /** Get the task strings to be parsed. */
  public List<String> getTasks() {
    return tasks;
  }

  /**
   * Get the starting pose for odometry reset.
   *
   * <p>Expected format: [x_meters, y_meters, rotation_degrees]
   *
   * @return The starting pose, or origin if invalid
   */
  public Pose2d getStartingPose() {
    if (startingPose == null || startingPose.length != 3) {
      return new Pose2d();
    }
    return new Pose2d(startingPose[0], startingPose[1], Rotation2d.fromDegrees(startingPose[2]));
  }

  /**
   * Load a routine from a JSON file.
   *
   * @param jsonFile The JSON file to load
   * @return The parsed routine
   * @throws IOException If file cannot be read or parsed
   */
  public static Routine loadFromJson(File jsonFile) throws IOException {
    ObjectMapper mapper = new ObjectMapper();
    return mapper.readValue(jsonFile, Routine.class);
  }
}
