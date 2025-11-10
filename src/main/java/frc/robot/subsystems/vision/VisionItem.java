package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionItem {

  public String name;
  public Transform3d cameraToRobot;
  public VisionEnums.CAMERA_TYPE cameraType;

  public VisionItem(String name, Transform3d cameraToRobot, VisionEnums.CAMERA_TYPE cameraType) {
    this.name = name;
    this.cameraToRobot = cameraToRobot;
    this.cameraType = cameraType;
  }
}
