package frc.robot.util.Kapok.Roots.factories;

import frc.robot.util.Kapok.Roots.core.Task;
import java.util.Optional;

/**
 * Factory for creating tasks from action and location strings.
 *
 * <p>This is a functional interface that creates a Task from an action type and location
 * identifier. Typically used with TaskRegistry to map task types to factory functions.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * // Register a PICKUP task factory
 * registry.register("PICKUP", (location) -> {
 *   Pose2d targetPose = fieldConstants.getLocation(location).orElseThrow();
 *   Pose2d approachPose = TaskParserUtil.calculateApproachPose(targetPose, 0.5);
 *   int tagID = fieldConstants.getAprilTagID(location).orElse(-1);
 *   return new Task("PICKUP", location, approachPose, targetPose, tagID);
 * });
 * }</pre>
 */
@FunctionalInterface
public interface TaskFactory {

  /**
   * Create a task from a location identifier.
   *
   * @param location The location identifier (e.g., "E2", "FEEDER_L")
   * @return The created task, or empty if unable to create
   */
  Optional<Task> createTask(String location);
}
