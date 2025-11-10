package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Reefscape.ReefscapeStateMachine.CurrentState;
import frc.robot.util.Kapok.Reefscape.ReefscapeStateMachine.WantedState;

public class AutoExecutionCommand extends Command {
  private final RobotContainer robot;
  private final ReefscapeStateMachine stateMachine;
  private final Pose2d startingPose;

  public AutoExecutionCommand(
      RobotContainer robot, ReefscapeStateMachine stateMachine, Pose2d startingPose) {
    this.robot = robot;
    this.stateMachine = stateMachine;
    this.startingPose = startingPose;

    // Require drive subsystem so default command doesn't interfere
    // Pathfinding command will be manually executed, not scheduled
    addRequirements(robot.drive);
  }

  @Override
  public void initialize() {
    // reset odom to starting pose
    robot.drive.setPose(startingPose);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Auto/StartingPose", startingPose);
    org.littletonrobotics.junction.Logger.recordOutput("Kapok/Auto/Initialized", true);

    // start the state machine
    stateMachine.setWantedState(WantedState.EXECUTE_AUTO);
  }

  @Override
  public void execute() {
    // update state machine
    stateMachine.update();
  }

  @Override
  public boolean isFinished() {
    return stateMachine.getCurrentState() == CurrentState.COMPLETE;
  }

  @Override
  public void end(boolean interrupted) {
    stateMachine.setWantedState(WantedState.IDLE);
  }
}
