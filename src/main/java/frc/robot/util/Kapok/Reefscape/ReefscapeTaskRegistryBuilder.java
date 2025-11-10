package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Kapok.Roots.core.Task;
import frc.robot.util.Kapok.Roots.factories.FieldConstants;
import frc.robot.util.Kapok.Roots.factories.TaskRegistry;
import frc.robot.util.Kapok.Roots.parsing.TaskParserUtil;
import java.util.Optional;

/**
 * Builds a TaskRegistry for Reefscape with all game specific task types.
 *
 * <p>This class registers factory functions for:
 *
 * <ul>
 *   <li>PICKUP - Pick up game pieces from feeder stations
 *   <li>SCORE - Score at specific reef locations
 *   <li>SCORE_AUTO - Dynamic scoring (resolved at runtime)
 * </ul>
 */
public class ReefscapeTaskRegistryBuilder {

  private final FieldConstants fieldConstants;

  public ReefscapeTaskRegistryBuilder(FieldConstants fieldConstants) {
    this.fieldConstants = fieldConstants;
  }

  /**
   * Build and configure a TaskRegistry with all Reefscape task types.
   *
   * @return Configured task registry
   */
  public TaskRegistry build() {
    TaskRegistry registry = new TaskRegistry();

    // Register PICKUP task factory
    registry.register("PICKUP", this::createPickupTask);

    // Register SCORE task factory
    registry.register("SCORE", this::createScoreTask);

    return registry;
  }

  /**
   * Create a PICKUP task.
   *
   * @param location The location (e.g., "FEEDER_L", "FEEDER_R")
   * @return The pickup task
   */
  private Optional<Task> createPickupTask(String location) {
    // Get target pose from field constants
    Optional<Pose2d> targetPoseOpt = fieldConstants.getLocation(location);
    if (targetPoseOpt.isEmpty()) {
      return Optional.empty();
    }
    Pose2d targetPose = targetPoseOpt.get();

    // Calculate approach pose (0.8m in front)
    Pose2d approachPose = TaskParserUtil.calculateApproachPose(targetPose, 0.8);

    // Get AprilTag ID
    int tagID = fieldConstants.getAprilTagID(location).orElse(-1);

    return Optional.of(new Task("PICKUP", location, approachPose, targetPose, tagID));
  }

  /**
   * Create a SCORE task.
   *
   * @param location The location (e.g., "E2", "F3", "AUTO")
   * @return The score task
   */
  private Optional<Task> createScoreTask(String location) {
    // Handle dynamic scoring
    if (location.equalsIgnoreCase("AUTO")) {
      return Optional.of(createDynamicScoreTask());
    }

    // Get target pose from field constants
    Optional<Pose2d> targetPoseOpt = fieldConstants.getLocation(location);
    if (targetPoseOpt.isEmpty()) {
      return Optional.empty();
    }
    Pose2d targetPose = targetPoseOpt.get();

    // Calculate approach pose (0.6m in front)
    Pose2d approachPose = TaskParserUtil.calculateApproachPose(targetPose, 0.6);

    // Get AprilTag ID
    int tagID = fieldConstants.getAprilTagID(location).orElse(-1);

    return Optional.of(new Task("SCORE", location, approachPose, targetPose, tagID));
  }

  /**
   * Create a dynamic scoring task (will be resolved at runtime).
   *
   * @return Placeholder task for dynamic scoring
   */
  private Task createDynamicScoreTask() {
    // Placeholder poses - actual scoring will use strategy selector
    return new Task("SCORE_AUTO", "AUTO", new Pose2d(), new Pose2d(), -1);
  }
}
