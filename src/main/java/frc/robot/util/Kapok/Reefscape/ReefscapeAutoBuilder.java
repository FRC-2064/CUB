package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.core.Routine;
import frc.robot.util.Kapok.Roots.core.Task;
import frc.robot.util.Kapok.Roots.factories.FieldConstants;
import frc.robot.util.Kapok.Roots.factories.TaskRegistry;
import frc.robot.util.Kapok.Roots.parsing.TaskParser;
import java.util.List;

/**
 * Auto builder for Reefscape (2025) autonomous routines.
 *
 * <p>Builds autonomous commands from JSON routines using the Kapok.
 */
public class ReefscapeAutoBuilder {
  private final RobotContainer robot;
  private final TaskParser taskParser;

  /**
   * Create a Reefscape auto builder.
   *
   * @param robot The robot container
   */
  public ReefscapeAutoBuilder(RobotContainer robot) {
    this.robot = robot;

    // Setup field constants
    FieldConstants fieldConstants = new ReefscapeFieldConstants();

    // Build task registry with all Reefscape task types
    TaskRegistry registry = new ReefscapeTaskRegistryBuilder(fieldConstants).build();

    // Create parser
    this.taskParser = new TaskParser(registry);
  }

  /**
   * Build an autonomous command from a routine.
   *
   * @param routine The routine to execute
   * @return The autonomous command
   */
  public Command buildAutoCommand(Routine routine) {
    // Parse all task strings using registry-based parser
    List<Task> tasks = taskParser.parseTasks(routine.getTasks());

    // Create game-specific context
    ReefscapeContext context = new ReefscapeContext(tasks, robot);

    // Create state machine
    ReefscapeStateMachine stateMachine = new ReefscapeStateMachine(robot, context);

    // Return execution command
    return new AutoExecutionCommand(robot, stateMachine, routine.getStartingPose());
  }
}
