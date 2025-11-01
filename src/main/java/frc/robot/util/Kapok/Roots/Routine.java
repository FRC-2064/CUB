package frc.robot.util.Kapok.Roots;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.File;
import java.util.List;

public class Routine {
  @JsonProperty("name")
  private String name;

  @JsonProperty("startingPose")
  private double[] startingPose;

  @JsonProperty("tasks")
  private List<String> tasks;

  public String getName() {
    return name;
  }

  public List<String> getTasks() {
    return tasks;
  }

  public Pose2d getStartingPose() {
    if (startingPose == null || startingPose.length != 3) {
      return new Pose2d();
    }
    return new Pose2d(startingPose[0], startingPose[1], Rotation2d.fromDegrees(startingPose[2]));
  }

  public static Routine loadFromJson(File jsonFile) throws Exception {
    ObjectMapper m = new ObjectMapper();
    return m.readValue(jsonFile, Routine.class);
  }
}
