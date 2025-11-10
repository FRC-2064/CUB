package frc.robot.util.Kapok.Reefscape;

import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.core.Context;
import frc.robot.util.Kapok.Roots.core.StateMachine;
import frc.robot.util.Kapok.Roots.core.Task;
import frc.robot.util.Kapok.Roots.vision.VisionAlignmentHelper;
import org.littletonrobotics.junction.Logger;

public class ReefscapeStateMachine
    extends StateMachine<ReefscapeStateMachine.WantedState, ReefscapeStateMachine.CurrentState> {

  public enum WantedState {
    IDLE,
    EXECUTE_AUTO
  }

  public enum CurrentState {
    IDLE,
    PATHFINDING_TO_APPROACH,
    VISION_ALIGNING,
    EXECUTING_PICKUP,
    EXECUTING_SCORE,
    TRANSITIONING,
    COMPLETE
  }

  private final RobotContainer robot;
  private final Context context;
  private final ReefscapeVisionAlignment visionHelper;

  private boolean hasPathfindingStarted = false;
  private boolean hasVisionAlignmentStarted = false;

  public ReefscapeStateMachine(RobotContainer robot, Context context) {
    super("Auto/Reefscape", WantedState.IDLE, CurrentState.IDLE);
    this.robot = robot;
    this.context = context;
    this.visionHelper = new ReefscapeVisionAlignment(robot);
  }

  @Override
  protected CurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return CurrentState.IDLE;
      case EXECUTE_AUTO:
        if (context.isComplete()) {
          return CurrentState.COMPLETE;
        }

        Task currentTask = context.getCurrentTask();

        switch (currentTask.getCurrentState()) {
          case PATHFINDING:
            return CurrentState.PATHFINDING_TO_APPROACH;
          case VISION_ALIGNMENT:
            return CurrentState.VISION_ALIGNING;
          case EXECUTING:
            if (currentTask.getTaskType().equals("PICKUP")) {
              return CurrentState.EXECUTING_PICKUP;
            } else {
              return CurrentState.EXECUTING_SCORE;
            }
          case COMPLETE:
            return CurrentState.TRANSITIONING;
        }
        break;
    }
    return currentState;
  }

  @Override
  protected void applyState() {
    if (hasStateChanged()) {
      if (currentState != CurrentState.PATHFINDING_TO_APPROACH) {
        hasPathfindingStarted = false;
      }
      if (currentState != CurrentState.VISION_ALIGNING) {
        hasVisionAlignmentStarted = false;
      }
    }

    switch (currentState) {
      case IDLE:
        handleIdle();
        break;
      case PATHFINDING_TO_APPROACH:
        handlePathfinding();
        break;
      case VISION_ALIGNING:
        handleVisionAlginment();
        break;
      case EXECUTING_PICKUP:
        handlePickup();
        break;
      case EXECUTING_SCORE:
        handleScore();
        break;
      case TRANSITIONING:
        handleTransitioning();
        break;
      case COMPLETE:
        handleComplete();
        break;
    }
  }

  private void handleIdle() {
    // set superstructure to be idle
  }

  private void handlePathfinding() {
    // if pathfinding didnt start, start it
    if (hasPathfindingStarted) {
      // start
    }
    // check if pathfinding finished, if it did set the task phase to vision alignment
  }

  private void handleVisionAlginment() {
    Task task = context.getCurrentTask();

    VisionAlignmentHelper.AlignmentResult result =
        visionHelper.alignToTarget(task.getTargetAprilTagID(), task.getTargetPose());

    if (result.isAligned) {
      task.setCurrentState(Task.State.EXECUTING);
    } else if (!result.tagVisible) {
      Logger.recordOutput("Auto/VisionAlignment/TagNotVisible", true);
    }
  }

  private void handlePickup() {
    Task task = context.getCurrentTask();

    if (didEnterState(CurrentState.EXECUTING_PICKUP)) {
      // superstructure to some pickup state
    }

    // check the superstructure if it has coral
    task.setCurrentState(Task.State.COMPLETE);
  }

  private void handleScore() {
    Task task = context.getCurrentTask();

    if (didEnterState(CurrentState.EXECUTING_SCORE)) {
      // superstructure to some pickup state
    }

    // check the superstructure if scoring is done
    task.setCurrentState(Task.State.COMPLETE);
  }

  private void handleTransitioning() {
    if (context.hasNextTask()) {
      context.nextTask();
    } else {
      setWantedState(WantedState.IDLE);
    }
  }

  private void handleComplete() {
    // set to an idle state
  }
}
