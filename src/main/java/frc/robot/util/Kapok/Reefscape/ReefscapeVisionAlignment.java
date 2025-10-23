package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.util.Kapok.Roots.VisionAlignmentHelper;

public class ReefscapeVisionAlignment extends VisionAlignmentHelper {
    private final RobotContainer robot;

    public ReefscapeVisionAlignment(RobotContainer robot) {
        this.robot = robot;
    }
    
    @Override
    public AlignmentResult alignToTarget(int targetTagID, Pose2d targetPose) {
        // Get vision ovservations, check if its the target, create a pose from that
        return new AlignmentResult(false, new Pose2d(), false);
    }
}
