package frc.robot.util.Kapok.Roots.factories;

import frc.robot.util.Kapok.Roots.core.Task;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Registry of task types and their factory functions.
 *
 * <p>This registry maps task type strings (e.g., "PICKUP", "SCORE") to factory functions that
 * create tasks, for config based task parsing.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * TaskRegistry registry = new TaskRegistry();
 *
 * // Register PICKUP task factory
 * registry.register("PICKUP", location -> {
 *   Pose2d pose = fieldConstants.getLocation(location).orElseThrow();
 *   int tagID = fieldConstants.getAprilTagID(location).orElse(-1);
 *   return new Task("PICKUP", location, approachPose, pose, tagID);
 * });
 *
 * // Register SCORE task factory
 * registry.register("SCORE", location -> {
 *   // ... similar logic
 * });
 *
 * // Parse a task string
 * Task task = registry.createTask("PICKUP:FEEDER_L").orElseThrow();
 * }</pre>
 */
public class TaskRegistry {
  private final Map<String, TaskFactory> factories = new HashMap<>();

  /**
   * Register a task factory for a specific task type.
   *
   * @param taskType The task type (e.g., "PICKUP", "SCORE")
   * @param factory The factory function to create tasks of this type
   */
  public void register(String taskType, TaskFactory factory) {
    factories.put(taskType.toUpperCase(), factory);
  }

  /**
   * Create a task from a task string.
   *
   * <p>Format: "ACTION:LOCATION" (e.g., "PICKUP:FEEDER_L", "SCORE:E2")
   *
   * @param taskString The task string to parse
   * @return The created task, or empty if unable to parse/create
   */
  public Optional<Task> createTask(String taskString) {
    String[] parts = taskString.split(":", 2);
    if (parts.length != 2) {
      return Optional.empty();
    }

    String action = parts[0].trim().toUpperCase();
    String location = parts[1].trim();

    TaskFactory factory = factories.get(action);
    if (factory == null) {
      return Optional.empty();
    }

    return factory.createTask(location);
  }

  /**
   * Check if a task type is registered.
   *
   * @param taskType The task type to check
   * @return True if a factory exists for this type
   */
  public boolean isRegistered(String taskType) {
    return factories.containsKey(taskType.toUpperCase());
  }

  /**
   * Get all registered task types.
   *
   * @return Set of registered task type strings
   */
  public java.util.Set<String> getRegisteredTypes() {
    return java.util.Collections.unmodifiableSet(factories.keySet());
  }
}
