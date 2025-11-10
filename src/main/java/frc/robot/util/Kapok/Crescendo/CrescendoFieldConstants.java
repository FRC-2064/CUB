package frc.robot.util.Kapok.Crescendo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Kapok.Roots.factories.FieldConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Field constants for the 2024 Crescendo game.
 *
 * <p>Includes locations for: - Speaker positions (blue/red alliance) - Amp positions - Note staging
 * locations - Feeder stations
 */
public class CrescendoFieldConstants implements FieldConstants {

  private static final Map<String, Pose2d> LOCATIONS = new HashMap<>();
  private static final Map<String, Integer> TAG_IDS = new HashMap<>();

  static {
    // Blue Alliance Speaker (center point, robot faces tag 7)
    LOCATIONS.put("SPEAKER_BLUE", new Pose2d(0.0, 5.55, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("SPEAKER_BLUE", 7);

    // Red Alliance Speaker (center point, robot faces tag 4)
    LOCATIONS.put("SPEAKER_RED", new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(180.0)));
    TAG_IDS.put("SPEAKER_RED", 4);

    // Blue Alliance Amp (precise position, robot faces tag 6)
    LOCATIONS.put("AMP_BLUE", new Pose2d(1.84, 7.0, Rotation2d.fromDegrees(90.0)));
    TAG_IDS.put("AMP_BLUE", 6);

    // Red Alliance Amp (precise position, robot faces tag 5)
    LOCATIONS.put("AMP_RED", new Pose2d(14.7, 7.0, Rotation2d.fromDegrees(90.0)));
    TAG_IDS.put("AMP_RED", 5);

    // Centerline Notes (8 notes on centerline, approximate positions)
    LOCATIONS.put("NOTE_C1", new Pose2d(8.27, 0.75, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("NOTE_C1", -1); // No AprilTag for notes

    LOCATIONS.put("NOTE_C2", new Pose2d(8.27, 2.43, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("NOTE_C2", -1);

    LOCATIONS.put("NOTE_C3", new Pose2d(8.27, 4.11, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("NOTE_C3", -1);

    LOCATIONS.put("NOTE_C4", new Pose2d(8.27, 5.78, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("NOTE_C4", -1);

    LOCATIONS.put("NOTE_C5", new Pose2d(8.27, 7.46, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("NOTE_C5", -1);

    // Blue Alliance Spike Notes (3 notes in front of speaker)
    LOCATIONS.put("NOTE_SPIKE_B1", new Pose2d(2.9, 4.1, Rotation2d.fromDegrees(60.0)));
    TAG_IDS.put("NOTE_SPIKE_B1", -1);

    LOCATIONS.put("NOTE_SPIKE_B2", new Pose2d(2.9, 5.55, Rotation2d.fromDegrees(0.0)));
    TAG_IDS.put("NOTE_SPIKE_B2", -1);

    LOCATIONS.put("NOTE_SPIKE_B3", new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(-60.0)));
    TAG_IDS.put("NOTE_SPIKE_B3", -1);

    // Red Alliance Spike Notes
    LOCATIONS.put("NOTE_SPIKE_R1", new Pose2d(13.64, 4.1, Rotation2d.fromDegrees(120.0)));
    TAG_IDS.put("NOTE_SPIKE_R1", -1);

    LOCATIONS.put("NOTE_SPIKE_R2", new Pose2d(13.64, 5.55, Rotation2d.fromDegrees(180.0)));
    TAG_IDS.put("NOTE_SPIKE_R2", -1);

    LOCATIONS.put("NOTE_SPIKE_R3", new Pose2d(13.64, 7.0, Rotation2d.fromDegrees(-120.0)));
    TAG_IDS.put("NOTE_SPIKE_R3", -1);
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
