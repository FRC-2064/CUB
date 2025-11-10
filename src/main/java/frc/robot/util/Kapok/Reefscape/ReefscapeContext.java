package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.core.Context;
import frc.robot.util.Kapok.Roots.core.Task;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Reefscape specific context for autonomous execution. Provides access to robot state, field state,
 * and game specific information.
 */
public class ReefscapeContext extends Context {
  private final RobotContainer robot;
  private final Set<String> scoredLocations;
  private String currentGamePiece = "coral"; // Default game piece

  /**
   * Create a new Reefscape context.
   *
   * @param tasks The list of tasks to execute
   * @param robot The robot container for accessing subsystems
   */
  public ReefscapeContext(List<Task> tasks, RobotContainer robot) {
    super(tasks);
    this.robot = robot;
    this.scoredLocations = new HashSet<>();
  }

  @Override
  public Pose2d getRobotPose() {
    return robot.drive.getPose();
  }

  /**
   * Get the robot container.
   *
   * @return The robot container
   */
  public RobotContainer getRobot() {
    return robot;
  }

  /**
   * Get the set of already-scored location IDs.
   *
   * @return Set of scored location IDs
   */
  public Set<String> getScoredLocations() {
    return scoredLocations;
  }

  /**
   * Mark a location as scored.
   *
   * @param locationId The location ID to mark as scored
   */
  public void addScoredLocation(String locationId) {
    scoredLocations.add(locationId);
  }

  /**
   * Get the current game piece being carried/manipulated.
   *
   * @return The game piece type (e.g., "coral", "algae")
   */
  public String getCurrentGamePiece() {
    // TODO: This should query the actual robot state to determine what piece we have
    // For now, returning a default
    return currentGamePiece;
  }

  /**
   * Set the current game piece type.
   *
   * @param gamePiece The game piece type
   */
  public void setCurrentGamePiece(String gamePiece) {
    this.currentGamePiece = gamePiece;
  }

  /**
   * Check if the robot currently has a game piece.
   *
   * @return true if robot has a game piece
   */
  public boolean hasGamePiece() {
    // TODO: This should query actual robot sensors
    // For now, assuming we have one if currentGamePiece is set
    return currentGamePiece != null && !currentGamePiece.isEmpty();
  }
}
