package frc.robot.util.Kapok.Roots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;

public abstract class VisionAlignmentHelper {
    public static class AlignmentResult {
        public final boolean isAligned;
        public final Pose2d correctedPose;
        public final boolean tagVisible;

        public AlignmentResult(boolean isAligned, Pose2d correctedPose, boolean tagVisible) {
            this.isAligned = isAligned;
            this.correctedPose = correctedPose;
            this.tagVisible = tagVisible;
        }
    }

    public abstract AlignmentResult alignToTarget(int targetTagID, Pose2d targetPose);

    protected boolean isWithinTolerance(Pose2d current, Pose2d target, double xToleranceMeters, double yToleranceMeters, Angle rotToleranceDegrees) {
        boolean x = current.getTranslation().getMeasureX().isNear(target.getMeasureX(), xToleranceMeters);
        boolean y = current.getTranslation().getMeasureY().isNear(target.getMeasureY(), yToleranceMeters);
        boolean rot = current.getRotation().getMeasure().isNear(target.getRotation().getMeasure(), rotToleranceDegrees);

        return x && y && rot;
    }
}
