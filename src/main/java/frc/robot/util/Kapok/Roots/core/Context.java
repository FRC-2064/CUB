package frc.robot.util.Kapok.Roots.core;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Base context class for autonomous execution.
 *
 * <p>The Context manages the task list and provides an interface for game-specific state. Game-
 * specific implementations should extend this class and add:
 *
 * <ul>
 *   <li>Robot pose access (via getRobotPose())
 *   <li>Game piece state (hasGamePiece(), etc.)
 *   <li>Field state (scored locations, objectives, etc.)
 *   <li>Subsystem readiness checks
 * </ul>
 *
 * <p>Example:
 *
 * <pre>{@code
 * public class ReefscapeContext extends Context {
 *   private final RobotContainer robot;
 *   private final Set<String> scoredLocations = new HashSet<>();
 *   private String currentGamePiece = "coral";
 *
 *   public ReefscapeContext(List<Task> tasks, RobotContainer robot) {
 *     super(tasks);
 *     this.robot = robot;
 *   }
 *
 *   @Override
 *   public Pose2d getRobotPose() {
 *     return robot.drive.getPose();
 *   }
 *
 *   public boolean hasGamePiece() { ... }
 *   public void addScoredLocation(String location) { scoredLocations.add(location); }
 * }
 * }</pre>
 */
public abstract class Context {
  private final List<Task> tasks;
  private int currentTaskIndex = 0;
  private Task currentTask;

  /**
   * Create a new context with a list of tasks.
   *
   * @param tasks The tasks to execute (will be copied to allow modification)
   */
  public Context(List<Task> tasks) {
    this.tasks = new ArrayList<>(tasks);
    if (!this.tasks.isEmpty()) {
      this.currentTask = this.tasks.get(0);
    }
  }

  /**
   * Get the current robot pose.
   *
   * <p>Must be implemented by game-specific context to provide access to drive subsystem pose.
   *
   * @return Current robot pose in field coordinates
   */
  public abstract Pose2d getRobotPose();

  /** Get the current task being executed. */
  public Task getCurrentTask() {
    return currentTask;
  }

  /**
   * Get all tasks (unmodifiable view).
   *
   * <p>To modify tasks, use replaceCurrentTask().
   */
  public List<Task> getTasks() {
    return Collections.unmodifiableList(tasks);
  }

  /** Get the index of the current task (0-based). */
  public int getCurrentTaskIndex() {
    return currentTaskIndex;
  }

  /**
   * Check if there are more tasks after the current one.
   *
   * @return True if not on the last task
   */
  public boolean hasNextTask() {
    return currentTaskIndex < tasks.size() - 1;
  }

  /**
   * Advance to the next task.
   *
   * <p>Does nothing if already on the last task.
   *
   * @return The new current task, or null if no next task
   */
  public Task nextTask() {
    if (!hasNextTask()) {
      return null;
    }
    currentTaskIndex++;
    currentTask = tasks.get(currentTaskIndex);
    return currentTask;
  }

  /**
   * Replace the current task with a new task.
   *
   * <p>This is useful for resolving dynamic tasks like "SCORE:AUTO" which determine the actual
   * target at runtime based on robot position and field state.
   *
   * @param newTask The new task to replace the current task
   */
  public void replaceCurrentTask(Task newTask) {
    if (currentTaskIndex < tasks.size()) {
      tasks.set(currentTaskIndex, newTask);
      currentTask = newTask;
    }
  }

  /**
   * Check if all tasks are complete.
   *
   * @return True if on the last task and it's complete
   */
  public boolean isComplete() {
    // Check if we're on the last task AND it's complete
    if (currentTaskIndex == tasks.size() - 1 && currentTask != null) {
      return currentTask.isComplete();
    }
    // Or if we're somehow past the end
    return currentTaskIndex >= tasks.size();
  }

  /** Get the total number of tasks. */
  public int getTaskCount() {
    return tasks.size();
  }
}
