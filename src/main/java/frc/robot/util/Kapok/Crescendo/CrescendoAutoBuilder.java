package frc.robot.util.Kapok.Crescendo;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.core.Routine;
import frc.robot.util.Kapok.Roots.core.Task;
import frc.robot.util.Kapok.Roots.factories.FieldConstants;
import frc.robot.util.Kapok.Roots.factories.TaskRegistry;
import frc.robot.util.Kapok.Roots.parsing.TaskParser;
import java.util.List;

/**
 * Auto builder for Crescendo (2024) autonomous routines.
 *
 * <p>Example of how to use Kapok for a shooting game with different scoring types:
 *
 * <ul>
 *   <li>PICKUP notes from field
 *   <li>SCORE_SPEAKER (IN_RANGE scoring - shoot from distance)
 *   <li>SCORE_AMP (POSITIONAL scoring - precise positioning)
 * </ul>
 *
 * <p>This is an EXAMPLE implementation showing the pattern. In a real 2024 robot, you would: - Add
 * CrescendoStateMachine similar to ReefscapeStateMachine - Add AutoExecutionCommand for execution -
 * Implement subsystem interactions (intake, shooter, etc.)
 */
public class CrescendoAutoBuilder {
  private final RobotContainer robot;
  private final TaskParser taskParser;

  /**
   * Create a Crescendo auto builder.
   *
   * @param robot The robot container
   */
  public CrescendoAutoBuilder(RobotContainer robot) {
    this.robot = robot;

    // Setup field constants
    FieldConstants fieldConstants = new CrescendoFieldConstants();

    // Build task registry with all Crescendo task types
    TaskRegistry registry = new CrescendoTaskRegistryBuilder(fieldConstants).build();

    // Create parser
    this.taskParser = new TaskParser(registry);
  }

  /**
   * Build an autonomous command from a routine.
   *
   * <p>Example routine JSON:
   *
   * <pre>{@code
   * {
   *   "name": "4 Note Auto",
   *   "startingPose": [1.5, 5.55, 0],
   *   "tasks": [
   *     "PICKUP:NOTE_SPIKE_B2",
   *     "SCORE_SPEAKER:SPEAKER_BLUE",
   *     "PICKUP:NOTE_SPIKE_B1",
   *     "SCORE_SPEAKER:SPEAKER_BLUE",
   *     "PICKUP:NOTE_SPIKE_B3",
   *     "SCORE_SPEAKER:SPEAKER_BLUE"
   *   ]
   * }
   * }</pre>
   *
   * @param routine The routine to execute
   * @return The autonomous command
   */
  public Command buildAutoCommand(Routine routine) {
    System.out.println("Building Crescendo auto command for: " + routine.getName());

    // Parse all task strings using registry-based parser
    List<Task> tasks = taskParser.parseTasks(routine.getTasks());
    System.out.println("Parsed " + tasks.size() + " tasks");

    // Create game-specific context
    CrescendoContext context = new CrescendoContext(tasks, robot);

    // Path constraints for autonomous movement
    PathConstraints constraints =
        new PathConstraints(
            3.0, // max velocity m/s
            3.0, // max acceleration m/s^2
            Units.degreesToRadians(540), // max angular velocity rad/s
            Units.degreesToRadians(720) // max angular acceleration rad/s^2
            );

    // Create a command that actually drives to each location
    return Commands.sequence(
            // Reset pose
            robot.drive.runOnce(
                () -> {
                  robot.drive.setPose(routine.getStartingPose());
                  System.out.println("=== CRESCENDO AUTO START ===");
                  System.out.println("Routine: " + routine.getName());
                  System.out.println("Starting pose: " + routine.getStartingPose());
                }),

            // Execute each task by driving to the location
            Commands.sequence(
                tasks.stream()
                    .map(
                        task ->
                            Commands.sequence(
                                // Print task info
                                Commands.runOnce(
                                    () -> {
                                      System.out.println(
                                          "\n--- Starting Task: "
                                              + task.getTaskType()
                                              + " at "
                                              + task.getLocation()
                                              + " ---");
                                      System.out.println(
                                          "  Current pose: " + robot.drive.getPose());
                                      System.out.println(
                                          "  Target approach: " + task.getApproachPose());
                                    }),

                                // Drive to approach pose (with timeout)
                                AutoBuilder.pathfindToPose(
                                        task.getApproachPose(), constraints, 0.0 // end velocity
                                        )
                                    .withTimeout(5.0), // 5 second timeout
                                Commands.runOnce(
                                    () -> {
                                      System.out.println("  ✓ Reached approach pose");
                                      System.out.println(
                                          "  Driving to target: " + task.getTargetPose());
                                    }),

                                // Drive to target pose (if different from approach)
                                Commands.either(
                                    AutoBuilder.pathfindToPose(
                                            task.getTargetPose(), constraints, 0.0)
                                        .withTimeout(5.0), // 5 second timeout
                                    Commands.none(),
                                    () -> !task.getApproachPose().equals(task.getTargetPose())),

                                // Execute the task action
                                Commands.runOnce(
                                    () -> {
                                      System.out.println(
                                          "  ✓ At target pose: " + robot.drive.getPose());

                                      // Update context and print what action is happening
                                      if (task.getTaskType().equals("PICKUP")) {
                                        context.addNote(task.getLocation());
                                        System.out.println("  ⚙ Intaking note...");
                                        System.out.println(
                                            "  ✓ Picked up note. Total notes: "
                                                + context.getNotesInRobot());
                                      } else if (task.getTaskType().contains("SCORE")) {
                                        String scoredIn =
                                            task.getTaskType().contains("SPEAKER")
                                                ? "SPEAKER"
                                                : "AMP";
                                        System.out.println("  ⚙ Shooting into " + scoredIn + "...");
                                        context.removeNote(scoredIn);
                                      }
                                    }),

                                // Action delay - 1 second for shooting, 0.5 seconds for pickup
                                Commands.either(
                                    Commands.sequence(
                                        Commands.waitSeconds(1.0), // Shooting takes 1 second
                                        Commands.runOnce(
                                            () -> {
                                              System.out.println(
                                                  "  ✓ Shot complete! Total scores: "
                                                      + context.getTotalScores());
                                            })),
                                    Commands.sequence(
                                        Commands.waitSeconds(0.5), // Pickup takes 0.5 seconds
                                        Commands.runOnce(
                                            () -> System.out.println("  ✓ Pickup complete!"))),
                                    () -> task.getTaskType().contains("SCORE"))))
                    .toArray(Command[]::new)),

            // Final summary
            robot.drive.runOnce(
                () -> {
                  System.out.println("\n=== CRESCENDO AUTO COMPLETE ===");
                  System.out.println("Final pose: " + robot.drive.getPose());
                  System.out.println("Speaker scores: " + context.getSpeakerScores());
                  System.out.println("Amp scores: " + context.getAmpScores());
                  System.out.println("Total scores: " + context.getTotalScores());
                }))
        .withName("CrescendoAuto-" + routine.getName());
  }

  /**
   * Get the task parser (useful for testing).
   *
   * @return The task parser
   */
  public TaskParser getTaskParser() {
    return taskParser;
  }
}
