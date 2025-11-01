package frc.robot.util.Kapok.Roots;

import edu.wpi.first.math.geometry.Pose2d;

public class Task {

  public enum Phase {
    PATHFINDING,
    VISION_ALIGNMENT,
    EXECUTING,
    COMPLETE,
  }

  private final String taskType;
  private final String location;
  private final Pose2d approachPose;
  private final Pose2d targetPose;
  private final int targetAprilTagID;

  private Phase currentPhase = Phase.PATHFINDING;

  public Task(
      String taskType,
      String location,
      Pose2d approachPose,
      Pose2d targetPose,
      int targetAprilTagID) {
    this.taskType = taskType;
    this.location = location;
    this.approachPose = approachPose;
    this.targetPose = targetPose;
    this.targetAprilTagID = targetAprilTagID;
  }

  public String getTaskType() {
    return taskType;
  }

  public String getLocation() {
    return location;
  }

  public Pose2d getApproachPose() {
    return approachPose;
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public int getTargetAprilTagID() {
    return targetAprilTagID;
  }

  public Phase getCurrentPhase() {
    return currentPhase;
  }

  public void setCurrentPhase(Phase phase) {
    currentPhase = phase;
  }

  public boolean complete() {
    return currentPhase == Phase.COMPLETE;
  }
}
