package frc.robot.util.Kapok.Roots.parsing;

import frc.robot.util.Kapok.Roots.core.Task;
import frc.robot.util.Kapok.Roots.factories.TaskRegistry;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Main task parser that delegates to a TaskRegistry.
 *
 * <p>This parser uses a TaskRegistry to parse task strings. Game-specific code should:
 *
 * <ol>
 *   <li>Create a TaskRegistry
 *   <li>Register task type factories (PICKUP, SCORE, etc.)
 *   <li>Pass the registry to this parser
 *   <li>Use parseTask() to convert strings to Task objects
 * </ol>
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * // Setup (done once)
 * TaskRegistry registry = new TaskRegistry();
 * registry.register("PICKUP", location -> createPickupTask(location));
 * registry.register("SCORE", location -> createScoreTask(location));
 * TaskParser parser = new TaskParser(registry);
 *
 * // Usage
 * Task task = parser.parseTask("PICKUP:FEEDER_L").orElseThrow();
 * }</pre>
 */
public class TaskParser {
  private final TaskRegistry registry;
  private final BasicTaskParser basicParser;

  /**
   * Create a task parser with a registry.
   *
   * @param registry The task registry with registered factories
   */
  public TaskParser(TaskRegistry registry) {
    this.registry = registry;
    this.basicParser = new BasicTaskParser();
  }

  /**
   * Parse a task string into a Task object.
   *
   * <p>First tries basic commands (DELAY, POSE), then delegates to the registry for game-specific
   * tasks.
   *
   * @param taskString The task string (e.g., "PICKUP:FEEDER_L", "DELAY:1000")
   * @return The parsed task, or empty if unable to parse
   */
  public Optional<Task> parseTask(String taskString) {
    // Try basic commands first (DELAY, POSE)
    Task basicTask = basicParser.tryParse(taskString);
    if (basicTask != null) {
      return Optional.of(basicTask);
    }

    // Delegate to registry for game-specific tasks
    return registry.createTask(taskString);
  }

  /**
   * Parse multiple task strings.
   *
   * @param taskStrings List of task strings
   * @return List of parsed tasks
   * @throws IllegalArgumentException if any task cannot be parsed
   */
  public List<Task> parseTasks(List<String> taskStrings) {
    List<Task> tasks = new ArrayList<>();
    for (String taskString : taskStrings) {
      Task task =
          parseTask(taskString)
              .orElseThrow(
                  () -> new IllegalArgumentException("Unable to parse task: " + taskString));
      tasks.add(task);
    }
    return tasks;
  }

  /**
   * Get the underlying registry (for inspection/debugging).
   *
   * @return The task registry
   */
  public TaskRegistry getRegistry() {
    return registry;
  }
}
