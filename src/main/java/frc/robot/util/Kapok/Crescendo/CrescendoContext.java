package frc.robot.util.Kapok.Crescendo;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.core.Context;
import frc.robot.util.Kapok.Roots.core.Task;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Crescendo (2024) game-specific context for autonomous execution.
 *
 * <p>Tracks game-specific state:
 *
 * <ul>
 *   <li>Number of notes in robot
 *   <li>Notes that have been picked up from the field
 *   <li>Score counts (speaker vs amp)
 * </ul>
 */
public class CrescendoContext extends Context {

  private final RobotContainer robot;

  // Game state tracking
  private int notesInRobot = 0;
  private final Set<String> pickedUpNotes = new HashSet<>();
  private int speakerScores = 0;
  private int ampScores = 0;

  /**
   * Create a Crescendo autonomous context.
   *
   * @param tasks The list of tasks to execute
   * @param robot The robot container for accessing subsystems
   */
  public CrescendoContext(List<Task> tasks, RobotContainer robot) {
    super(tasks);
    this.robot = robot;
  }

  @Override
  public Pose2d getRobotPose() {
    return robot.drive.getPose();
  }

  // ==================== Game State Methods ====================

  /**
   * Check if the robot currently has a note.
   *
   * @return true if robot has at least one note
   */
  public boolean hasNote() {
    return notesInRobot > 0;
  }

  /**
   * Get the number of notes in the robot.
   *
   * @return Number of notes
   */
  public int getNotesInRobot() {
    return notesInRobot;
  }

  /**
   * Add a note to the robot (called after successful pickup).
   *
   * @param noteLocation The location ID of the picked up note
   */
  public void addNote(String noteLocation) {
    notesInRobot++;
    pickedUpNotes.add(noteLocation);
  }

  /**
   * Remove a note from the robot (called after scoring).
   *
   * @param scoredIn Where the note was scored ("SPEAKER" or "AMP")
   */
  public void removeNote(String scoredIn) {
    if (notesInRobot > 0) {
      notesInRobot--;

      if ("SPEAKER".equals(scoredIn)) {
        speakerScores++;
      } else if ("AMP".equals(scoredIn)) {
        ampScores++;
      }
    }
  }

  /**
   * Check if a note location has already been picked up.
   *
   * @param noteLocation The location ID to check
   * @return true if already picked up
   */
  public boolean isNotePickedUp(String noteLocation) {
    return pickedUpNotes.contains(noteLocation);
  }

  /**
   * Get the set of all picked up note locations.
   *
   * @return Set of note location IDs
   */
  public Set<String> getPickedUpNotes() {
    return new HashSet<>(pickedUpNotes);
  }

  /**
   * Get the number of speaker scores.
   *
   * @return Speaker score count
   */
  public int getSpeakerScores() {
    return speakerScores;
  }

  /**
   * Get the number of amp scores.
   *
   * @return Amp score count
   */
  public int getAmpScores() {
    return ampScores;
  }

  /**
   * Get the total number of scores.
   *
   * @return Total score count
   */
  public int getTotalScores() {
    return speakerScores + ampScores;
  }

  /**
   * Get the robot container for accessing subsystems.
   *
   * @return The robot container
   */
  public RobotContainer getRobot() {
    return robot;
  }
}
