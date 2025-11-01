package frc.robot.util.Kapok.Roots;

import java.util.List;

public class Context {
  private final List<Task> tasks;
  private int currentTaskIndex = 0;
  private Task currentTask;

  public Context(List<Task> tasks) {
    this.tasks = tasks;
    if (!tasks.isEmpty()) {
      this.currentTask = tasks.get(0);
    }
  }

  public Task getCurrentTask() {
    return currentTask;
  }

  public List<Task> getTasks() {
    return tasks;
  }

  public int getCurrentTaskIndex() {
    return currentTaskIndex;
  }

  public boolean hasNextTask() {
    return currentTaskIndex < tasks.size() - 1;
  }

  public void nextTask() {
    if (!hasNextTask()) {
      return;
    }
    currentTaskIndex++;
    currentTask = tasks.get(currentTaskIndex);
  }

  public boolean completed() {
    return currentTaskIndex >= tasks.size() - 1;
  }
}
