package frc.robot.util.Kapok.Roots.factories;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Interface for field location constants.
 *
 * <p>Game-specific implementations provide field locations, AprilTag IDs, and other spatial data
 * for the current year's game. This separates field data from task parsing logic.
 *
 * <p>Example implementation:
 *
 * <pre>{@code
 * public class ReefscapeFieldConstants implements FieldConstants {
 *   private static final Map<String, Pose2d> LOCATIONS = new HashMap<>();
 *   private static final Map<String, Integer> TAG_IDS = new HashMap<>();
 *
 *   static {
 *     LOCATIONS.put("E2", new Pose2d(4.99, 2.82, Rotation2d.fromDegrees(120)));
 *     TAG_IDS.put("E2", 22);
 *     // ... more locations
 *   }
 *
 *   public Optional<Pose2d> getLocation(String name) {
 *     return Optional.ofNullable(LOCATIONS.get(name));
 *   }
 *
 *   public Optional<Integer> getAprilTagID(String name) {
 *     return Optional.ofNullable(TAG_IDS.get(name));
 *   }
 * }
 * }</pre>
 */
public interface FieldConstants {

  /**
   * Get a field location by name.
   *
   * @param name The location name (e.g., "E2", "FEEDER_L")
   * @return The pose of that location, or empty if not found
   */
  Optional<Pose2d> getLocation(String name);

  /**
   * Get the AprilTag ID associated with a location.
   *
   * @param name The location name
   * @return The AprilTag ID, or empty if no tag associated
   */
  Optional<Integer> getAprilTagID(String name);

  /**
   * Check if a location exists.
   *
   * @param name The location name
   * @return True if the location is defined
   */
  default boolean hasLocation(String name) {
    return getLocation(name).isPresent();
  }
}
