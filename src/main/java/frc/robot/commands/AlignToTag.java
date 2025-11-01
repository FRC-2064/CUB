package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Kapok.Reefscape.ReefscapeVisionAlignment;
import frc.robot.util.Kapok.Roots.VisionAlignmentHelper.AlignmentResult;

public class AlignToTag extends Command {
  private final Drive drive;
  private final ReefscapeVisionAlignment visionAlignment;
  private final int tagId;
  private final Pose2d targetPose;

  public AlignToTag(
      Drive drive,
      ReefscapeVisionAlignment visionAlignment,
      int tagId,
      Pose2d targetPose) {
    this.drive = drive;
    this.visionAlignment = visionAlignment;
    this.tagId = tagId;
    this.targetPose = targetPose;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    AlignmentResult result = visionAlignment.alignToTarget(tagId, targetPose);
    if (result.tagVisible) {
      drive.runVelocity(result.speeds);
    } else {
      drive.stop();
    }
  }

  @Override
  public boolean isFinished() {
    // The command should probably not end automatically when aligned
    // It should hold the position.
    // The user can add a timeout or a button release to end it.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
