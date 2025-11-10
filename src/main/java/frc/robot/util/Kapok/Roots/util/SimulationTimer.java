package frc.robot.util.Kapok.Roots.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A simple timer for simulation mode to mock time based operations.
 *
 * <p>This is useful for instantly completing actions in simulation that would normally require
 * waiting for hardware (like intake mechanisms, shooter spinup, etc.).
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * private final SimulationTimer pickupTimer = new SimulationTimer(0.5); // 500ms
 *
 * // In state machine:
 * if (!pickupTimer.isRunning()) {
 *   pickupTimer.start();
 * }
 *
 * if (pickupTimer.isComplete()) {
 *   // Pickup complete!
 *   pickupTimer.reset();
 * }
 * }</pre>
 */
public class SimulationTimer {
  private double startTime;
  private final double durationSeconds;
  private boolean running = false;

  /**
   * Create a simulation timer with a fixed duration.
   *
   * @param durationSeconds How long the timer should run (in seconds)
   */
  public SimulationTimer(double durationSeconds) {
    this.durationSeconds = durationSeconds;
  }

  /** Start or restart the timer. */
  public void start() {
    startTime = Timer.getFPGATimestamp();
    running = true;
  }

  /** Reset the timer to not running. */
  public void reset() {
    running = false;
  }

  /**
   * Check if the timer has completed its duration.
   *
   * @return True if the timer is running and duration has elapsed
   */
  public boolean isComplete() {
    if (!running) {
      return false;
    }
    return Timer.getFPGATimestamp() - startTime >= durationSeconds;
  }

  /**
   * Check if the timer is currently running.
   *
   * @return True if the timer has been started and not reset
   */
  public boolean isRunning() {
    return running;
  }

  /**
   * Get the elapsed time since the timer started.
   *
   * @return Elapsed time in seconds, or 0 if not running
   */
  public double getElapsedTime() {
    if (!running) {
      return 0;
    }
    return Timer.getFPGATimestamp() - startTime;
  }

  /**
   * Get the remaining time until completion.
   *
   * @return Remaining time in seconds, or 0 if not running or already complete
   */
  public double getRemainingTime() {
    if (!running) {
      return 0;
    }
    double remaining = durationSeconds - getElapsedTime();
    return Math.max(0, remaining);
  }

  /**
   * Get the configured duration.
   *
   * @return The duration in seconds
   */
  public double getDuration() {
    return durationSeconds;
  }

  /**
   * Get the progress as a percentage (0.0 to 1.0).
   *
   * @return Progress percentage, or 0 if not running
   */
  public double getProgress() {
    if (!running) {
      return 0;
    }
    double progress = getElapsedTime() / durationSeconds;
    return Math.min(1.0, progress);
  }
}
