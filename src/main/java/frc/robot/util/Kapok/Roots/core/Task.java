package frc.robot.util.Kapok.Roots.core;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Represents a single autonomous task with execution states.
 *
 * <p>A task progresses through satetes: PATHFINDING → VISION_ALIGNMENT → EXECUTING → COMPLETE
 *
 * <p>Tasks contain:
 *
 * <ul>
 *   <li>Type and location identifiers
 *   <li>Approach pose (where to pathfind to initially)
 *   <li>Target pose (final aligned position)
 *   <li>Optional AprilTag ID for vision alignment
 * </ul>
 */
public class Task {

  /** The states a task progresses through during execution. */
  public enum State {
    PATHFINDING,
    VISION_ALIGNMENT,
    EXECUTING,
    COMPLETE
  }

  private final String taskType;
  private final String location;
  private final Pose2d approachPose;
  private final Pose2d targetPose;
  private final int targetAprilTagID; // -1 if no vision alignment needed

  private State currentState = State.PATHFINDING;

  /**
   * Create a new task.
   *
   * @param taskType The type of task (e.g., "PICKUP", "SCORE", "DELAY")
   * @param location The location identifier (e.g., "E2", "FEEDER_L")
   * @param approachPose The pose to pathfind to initially
   * @param targetPose The final target pose after alignment
   * @param targetAprilTagID The AprilTag ID for vision alignment, or -1 for no vision
   */
  public Task(
      String taskType,
      String location,
      Pose2d approachPose,
      Pose2d targetPose,
      int targetAprilTagID) {
    this.taskType = taskType;
    this.location = location;
    this.approachPose = approachPose;
    this.targetPose = targetPose;
    this.targetAprilTagID = targetAprilTagID;
  }

  /** Get the task type (e.g., "PICKUP", "SCORE"). */
  public String getTaskType() {
    return taskType;
  }

  /** Get the location identifier (e.g., "E2", "FEEDER_L"). */
  public String getLocation() {
    return location;
  }

  /** Get the approach pose (where to pathfind to). */
  public Pose2d getApproachPose() {
    return approachPose;
  }

  /** Get the target pose (final aligned position). */
  public Pose2d getTargetPose() {
    return targetPose;
  }

  /**
   * Get the AprilTag ID for vision alignment.
   *
   * @return The tag ID, or -1 if no vision alignment needed
   */
  public int getTargetAprilTagID() {
    return targetAprilTagID;
  }

  /**
   * Check if this task requires vision alignment.
   *
   * @return True if vision alignment is needed
   */
  public boolean requiresVisionAlignment() {
    return targetAprilTagID != -1;
  }

  /**
   * Get the AprilTag ID as an Optional.
   *
   * @return The tag ID, or empty if no vision alignment needed
   */
  public Optional<Integer> getAprilTagID() {
    return targetAprilTagID == -1 ? Optional.empty() : Optional.of(targetAprilTagID);
  }

  /** Get the current execution state. */
  public State getCurrentState() {
    return currentState;
  }

  /** Set the current execution state. */
  public void setCurrentState(State state) {
    this.currentState = state;
  }

  /**
   * Check if the task is complete.
   *
   * @return True if in COMPLETE state
   */
  public boolean isComplete() {
    return currentState == State.COMPLETE;
  }

  @Override
  public String toString() {
    return String.format(
        "Task{type='%s', location='%s', state=%s, tagID=%d}",
        taskType, location, currentState, targetAprilTagID);
  }
}
