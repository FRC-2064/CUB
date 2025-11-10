package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionEnums.CAMERA_TYPE;

public class Cameras {
  public static final VisionItem LIMELIGHT_ONE =
      new VisionItem("limelight-one", Transform3d.kZero, CAMERA_TYPE.LIMELIGHT);
  public static final VisionItem LIMELIGHT_TWO =
      new VisionItem("limelight-two", Transform3d.kZero, CAMERA_TYPE.LIMELIGHT);
  public static final VisionItem PHOTONVISION_ONE =
      new VisionItem("pv-1", Transform3d.kZero, CAMERA_TYPE.PHOTONVISION);
  public static final VisionItem PHOTONVISION_TWO =
      new VisionItem("pv-2", Transform3d.kZero, CAMERA_TYPE.PHOTONVISION);
}
