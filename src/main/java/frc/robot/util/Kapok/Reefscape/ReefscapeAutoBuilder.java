package frc.robot.util.Kapok.Reefscape;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.Context;
import frc.robot.util.Kapok.Roots.Routine;
import frc.robot.util.Kapok.Roots.Task;

public class ReefscapeAutoBuilder {
    private final RobotContainer robot;

    public ReefscapeAutoBuilder(RobotContainer robot) {
        this.robot = robot;
    }

    public Command buildAutoCommand(Routine routine) {
        List<Task> tasks = new ArrayList<>();
        for (String taskString : routine.getTasks()) {
            tasks.add(ReefscapeTaskParser.parseTask(taskString));
        }

        Context context = new Context(tasks);

        ReefscapeStateMachine stateMachine = new ReefscapeStateMachine(robot, context);

        return new AutoExecutionCommand(robot, stateMachine, routine.getStartingPose());
    }
}
