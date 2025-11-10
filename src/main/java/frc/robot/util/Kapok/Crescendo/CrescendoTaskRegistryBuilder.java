package frc.robot.util.Kapok.Crescendo;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Kapok.Roots.core.Task;
import frc.robot.util.Kapok.Roots.factories.FieldConstants;
import frc.robot.util.Kapok.Roots.factories.TaskRegistry;
import frc.robot.util.Kapok.Roots.parsing.TaskParserUtil;
import java.util.Optional;

/**
 * Builds a TaskRegistry for Crescendo (2024) with all game-specific task types.
 *
 * <p>Demonstrates different scoring patterns:
 *
 * <ul>
 *   <li>PICKUP - Pick up notes from field (no vision alignment needed)
 *   <li>SCORE_SPEAKER - IN_RANGE scoring (shoot from distance, vision-based)
 *   <li>SCORE_AMP - POSITIONAL scoring (precise positioning required)
 * </ul>
 */
public class CrescendoTaskRegistryBuilder {

  private final FieldConstants fieldConstants;

  public CrescendoTaskRegistryBuilder(FieldConstants fieldConstants) {
    this.fieldConstants = fieldConstants;
  }

  /**
   * Build and configure a TaskRegistry with all Crescendo task types.
   *
   * @return Configured task registry
   */
  public TaskRegistry build() {
    TaskRegistry registry = new TaskRegistry();

    // Register task factories
    registry.register("PICKUP", this::createPickupTask);
    registry.register("SCORE_SPEAKER", this::createSpeakerScoreTask);
    registry.register("SCORE_AMP", this::createAmpScoreTask);

    System.out.println(
        "Crescendo TaskRegistry built. Registered types: " + registry.getRegisteredTypes());

    return registry;
  }

  /**
   * Create a PICKUP task for collecting notes.
   *
   * <p>Example: "PICKUP:NOTE_C1" - pickup centerline note 1
   *
   * @param location The note location (e.g., "NOTE_C1", "NOTE_SPIKE_B1")
   * @return The pickup task, or empty if location not found
   */
  private Optional<Task> createPickupTask(String location) {
    System.out.println("createPickupTask called with location: " + location);
    Optional<Pose2d> targetPoseOpt = fieldConstants.getLocation(location);
    System.out.println(
        "Field constants returned: " + (targetPoseOpt.isPresent() ? "FOUND" : "NOT FOUND"));
    if (targetPoseOpt.isEmpty()) {
      System.out.println("ERROR: Location '" + location + "' not found in field constants!");
      return Optional.empty();
    }
    Pose2d targetPose = targetPoseOpt.get();

    // For note pickup, approach from 0.5m away (no vision alignment needed)
    Pose2d approachPose = TaskParserUtil.calculateApproachPose(targetPose, 0.5);

    // Notes don't have AprilTags, so no vision alignment
    System.out.println("Successfully created PICKUP task for: " + location);
    return Optional.of(new Task("PICKUP", location, approachPose, targetPose, -1));
  }

  /**
   * Create a SCORE_SPEAKER task.
   *
   * <p>Example: "SCORE_SPEAKER:SPEAKER_BLUE" - score in blue alliance speaker
   *
   * <p>This demonstrates IN_RANGE scoring - robot can shoot from various distances/angles as long
   * as it's within range. Vision alignment can be used to aim at the speaker.
   *
   * @param location The speaker location (e.g., "SPEAKER_BLUE", "SPEAKER_RED")
   * @return The speaker scoring task, or empty if location not found
   */
  private Optional<Task> createSpeakerScoreTask(String location) {
    Optional<Pose2d> targetPoseOpt = fieldConstants.getLocation(location);
    if (targetPoseOpt.isEmpty()) {
      return Optional.empty();
    }
    Pose2d targetPose = targetPoseOpt.get();

    // For speaker scoring, we use vision alignment to aim
    // Approach pose can be flexible (IN_RANGE pattern)
    Pose2d approachPose = TaskParserUtil.calculateApproachPose(targetPose, 2.0);

    // Get AprilTag ID for vision alignment
    int tagID = fieldConstants.getAprilTagID(location).orElse(-1);

    // NOTE: In a real implementation, you might use a different task type
    // like "SCORE_SPEAKER_IN_RANGE" to indicate this uses IN_RANGE scoring
    // rather than positional scoring
    return Optional.of(new Task("SCORE_SPEAKER", location, approachPose, targetPose, tagID));
  }

  /**
   * Create a SCORE_AMP task.
   *
   * <p>Example: "SCORE_AMP:AMP_BLUE" - score in blue alliance amp
   *
   * <p>This demonstrates POSITIONAL scoring - robot must be at a precise position to score in the
   * amp.
   *
   * @param location The amp location (e.g., "AMP_BLUE", "AMP_RED")
   * @return The amp scoring task, or empty if location not found
   */
  private Optional<Task> createAmpScoreTask(String location) {
    Optional<Pose2d> targetPoseOpt = fieldConstants.getLocation(location);
    if (targetPoseOpt.isEmpty()) {
      return Optional.empty();
    }
    Pose2d targetPose = targetPoseOpt.get();

    // For amp scoring, precise positioning is required
    // Approach from 0.6m away
    Pose2d approachPose = TaskParserUtil.calculateApproachPose(targetPose, 0.6);

    // Get AprilTag ID for vision alignment
    int tagID = fieldConstants.getAprilTagID(location).orElse(-1);

    return Optional.of(new Task("SCORE_AMP", location, approachPose, targetPose, tagID));
  }
}
