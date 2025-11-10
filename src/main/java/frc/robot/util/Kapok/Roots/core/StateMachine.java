package frc.robot.util.Kapok.Roots.core;

import org.littletonrobotics.junction.Logger;

/**
 * Generic state machine with wanted/current state pattern.
 *
 * <p>This state machine uses two state enums:
 *
 * <ul>
 *   <li><b>Wanted State</b>: High-level intent (e.g., IDLE, EXECUTE_AUTO)
 *   <li><b>Current State</b>: Actual execution state (e.g., PATHFINDING, VISION_ALIGNING, etc.)
 * </ul>
 *
 * <p>The state machine automatically logs state transitions to AdvantageKit.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * public class MyStateMachine extends StateMachine<WantedState, CurrentState> {
 *   public enum WantedState { IDLE, EXECUTE_AUTO }
 *   public enum CurrentState { IDLE, PATHFINDING, EXECUTING, COMPLETE }
 *
 *   public MyStateMachine() {
 *     super("Kapok/MyAuto", WantedState.IDLE, CurrentState.IDLE);
 *   }
 *
 *   @Override
 *   protected CurrentState handleStateTransitions() {
 *     switch (wantedState) {
 *       case EXECUTE_AUTO:
 *         // State transition logic
 *         return CurrentState.PATHFINDING;
 *       default:
 *         return CurrentState.IDLE;
 *     }
 *   }
 *
 *   @Override
 *   protected void applyState() {
 *     switch (currentState) {
 *       case PATHFINDING:
 *         // Execute pathfinding logic
 *         break;
 *       // ... other states
 *     }
 *   }
 * }
 * }</pre>
 *
 * @param <W> The Wanted State enum type
 * @param <C> The Current State enum type
 */
public abstract class StateMachine<W extends Enum<W>, C extends Enum<C>> {
  protected W wantedState;
  protected C currentState;
  protected C previousState;

  protected final String logKey;

  /**
   * Create a state machine.
   *
   * @param logKey The AdvantageKit log key (e.g., "Kapok/Reefscape/Auto")
   * @param initialWantedState Initial wanted state (usually IDLE)
   * @param initialCurrentState Initial current state (usually IDLE)
   */
  public StateMachine(String logKey, W initialWantedState, C initialCurrentState) {
    this.logKey = logKey;
    this.wantedState = initialWantedState;
    this.currentState = initialCurrentState;
    this.previousState = initialCurrentState;
  }

  /**
   * Update the state machine.
   *
   * <p>Call this method periodically (e.g., in a Command's execute() method). It:
   *
   * <ol>
   *   <li>Saves the current state as previous
   *   <li>Calculates the new current state via handleStateTransitions()
   *   <li>Applies the current state via applyState()
   *   <li>Logs states to AdvantageKit
   * </ol>
   */
  public void update() {
    previousState = currentState;
    currentState = handleStateTransitions();
    applyState();

    // Log state to AdvantageKit
    Logger.recordOutput(logKey + "/WantedState", wantedState.toString());
    Logger.recordOutput(logKey + "/CurrentState", currentState.toString());
    Logger.recordOutput(logKey + "/PreviousState", previousState.toString());
  }

  /**
   * Decide the next current state based on wanted state and conditions.
   *
   * <p>This is where state transition logic lives. Examine the wanted state, task phases, sensor
   * inputs, etc. and return the appropriate next current state.
   *
   * @return The next current state
   */
  protected abstract C handleStateTransitions();

  /**
   * Apply the current state.
   *
   * <p>This is where state execution logic lives. Based on the current state, command the robot to
   * perform actions (run pathfinding, apply vision speeds, trigger mechanisms, etc.).
   */
  protected abstract void applyState();

  /**
   * Check if the state changed this update cycle.
   *
   * @return True if currentState != previousState
   */
  protected boolean hasStateChanged() {
    return currentState != previousState;
  }

  /**
   * Check if we just entered a specific state.
   *
   * @param state The state to check
   * @return True if we're in that state now but weren't last cycle
   */
  protected boolean didEnterState(C state) {
    return currentState == state && previousState != state;
  }

  /**
   * Check if we just exited a specific state.
   *
   * @param state The state to check
   * @return True if we were in that state last cycle but aren't now
   */
  protected boolean didExitState(C state) {
    return previousState == state && currentState != state;
  }

  /** Set the wanted state (high-level intent). */
  public void setWantedState(W state) {
    this.wantedState = state;
  }

  /** Get the current state. */
  public C getCurrentState() {
    return currentState;
  }

  /** Get the wanted state. */
  public W getWantedState() {
    return wantedState;
  }

  /**
   * Set the current state directly.
   *
   * <p>Prefer using wanted state and handleStateTransitions() for normal operation.
   */
  protected void setCurrentState(C state) {
    this.currentState = state;
  }
}
