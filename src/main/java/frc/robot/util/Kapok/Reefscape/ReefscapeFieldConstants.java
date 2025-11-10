package frc.robot.util.Kapok.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Kapok.Roots.factories.FieldConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Field constants for the 2025 Reefscape game.
 *
 * <p>Contains all field locations for reef scoring positions and feeder stations, along with their
 * associated AprilTag IDs.
 */
public class ReefscapeFieldConstants implements FieldConstants {

  private static final Map<String, Pose2d> LOCATIONS = new HashMap<>();
  private static final Map<String, Integer> TAG_IDS = new HashMap<>();

  static {
    // Reef scoring locations (coordinates from 2025-Comp Constants.java)
    // E and F positions share the same field location, level is handled by superstructure

    Rotation2d eRotation = Rotation2d.fromDegrees(120.0);
    Rotation2d fRotation = Rotation2d.fromDegrees(120.0);

    // CORAL_LOCATION_E (tag 22)
    LOCATIONS.put("E2", new Pose2d(4.993620808, 2.822371924, eRotation));
    LOCATIONS.put("E3", new Pose2d(4.993620808, 2.822371924, eRotation));
    TAG_IDS.put("E2", 22);
    TAG_IDS.put("E3", 22);

    // CORAL_LOCATION_F (tag 22)
    LOCATIONS.put("F2", new Pose2d(5.279409192, 2.987371924, fRotation));
    LOCATIONS.put("F3", new Pose2d(5.279409192, 2.987371924, fRotation));
    TAG_IDS.put("F2", 22);
    TAG_IDS.put("F3", 22);

    // Feeder stations
    // FEEDER_LOCATION_LEFT (tag 13)
    LOCATIONS.put("FEEDER_L", new Pose2d(1.123621854, 7.021460172, Rotation2d.fromDegrees(126.0)));
    TAG_IDS.put("FEEDER_L", 13);

    // FEEDER_LOCATION_RIGHT (tag 12)
    LOCATIONS.put("FEEDER_R", new Pose2d(1.123621854, 1.030339828, Rotation2d.fromDegrees(234.0)));
    TAG_IDS.put("FEEDER_R", 12);

    // TODO: Add remaining reef locations (A, B, C, D, G, H, I, J, K, L)
  }

  @Override
  public Optional<Pose2d> getLocation(String name) {
    return Optional.ofNullable(LOCATIONS.get(name));
  }

  @Override
  public Optional<Integer> getAprilTagID(String name) {
    return Optional.ofNullable(TAG_IDS.get(name));
  }
}
