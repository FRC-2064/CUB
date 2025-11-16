package frc.robot.util.Kapok.Roots.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.*;
import java.util.function.Predicate;
import java.util.stream.Collectors;

/**
 * Consolidated scoring utilities for autonomous routines.
 *
 * <p>Provides common scoring patterns, location management, and distance calculations used across
 * different FRC games.
 */
public class ScoringHelper {

  /** Types of scoring mechanisms used in FRC games. */
  public enum ScoringType {
    /**
     * Drive to a specific pose and score. Example: 2025 Reefscape coral stations (precise
     * positioning required)
     */
    POSITIONAL,

    /**
     * Get within range and score when ready. Example: 2024 Crescendo speaker shots (vision based,
     * distance checked)
     */
    IN_RANGE,

    /**
     * Score while driving through a zone. Example: 2024 Crescendo shooting while passing speaker
     */
    DRIVE_BY
  }

  /**
   * Represents a scoring location on the field.
   *
   * <p>This is a simple data class that can be extended for game-specific needs (e.g., add a level
   * field for Reefscape, or a target type for Crescendo).
   */
  public static class ScoringLocation {
    private final String id;
    private final Pose2d targetPose;
    private final int aprilTagId;

    /**
     * Creates a scoring location.
     *
     * @param id Unique identifier (e.g., "F2", "SPEAKER_CENTER")
     * @param targetPose The target pose for scoring
     * @param aprilTagId The AprilTag ID associated with this location (-1 if none)
     */
    public ScoringLocation(String id, Pose2d targetPose, int aprilTagId) {
      this.id = id;
      this.targetPose = targetPose;
      this.aprilTagId = aprilTagId;
    }

    public String getId() {
      return id;
    }

    public Pose2d getTargetPose() {
      return targetPose;
    }

    public int getAprilTagId() {
      return aprilTagId;
    }

    /**
     * Calculate distance from a pose to this location.
     *
     * @param fromPose The pose to measure from
     * @return Distance in meters
     */
    public double getDistanceFrom(Pose2d fromPose) {
      return fromPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    @Override
    public String toString() {
      return String.format(
          "ScoringLocation{id='%s', pose=%s, tagId=%d}", id, targetPose, aprilTagId);
    }
  }

  /**
   * Calculate distance between two poses.
   *
   * @param pose1 First pose
   * @param pose2 Second pose
   * @return Distance in meters
   */
  public static double getDistance(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  /**
   * Find the closest location to a reference pose.
   *
   * @param locations List of locations to search
   * @param fromPose Reference pose
   * @return The closest location, or empty if list is empty
   */
  public static <T extends ScoringLocation> Optional<T> findClosest(
      List<T> locations, Pose2d fromPose) {
    return locations.stream().min(Comparator.comparingDouble(loc -> loc.getDistanceFrom(fromPose)));
  }

  /**
   * Sort locations by distance from a reference pose (closest first).
   *
   * @param locations List of locations to sort
   * @param fromPose Reference pose
   * @return New list sorted by distance (ascending)
   */
  public static <T extends ScoringLocation> List<T> sortByDistance(
      List<T> locations, Pose2d fromPose) {
    return locations.stream()
        .sorted(Comparator.comparingDouble(loc -> loc.getDistanceFrom(fromPose)))
        .collect(Collectors.toList());
  }

  /**
   * Filter locations within a maximum distance.
   *
   * @param locations List of locations to filter
   * @param fromPose Reference pose
   * @param maxDistanceMeters Maximum distance in meters
   * @return List of locations within range, sorted by distance
   */
  public static <T extends ScoringLocation> List<T> filterByDistance(
      List<T> locations, Pose2d fromPose, double maxDistanceMeters) {
    return locations.stream()
        .filter(loc -> loc.getDistanceFrom(fromPose) <= maxDistanceMeters)
        .sorted(Comparator.comparingDouble(loc -> loc.getDistanceFrom(fromPose)))
        .collect(Collectors.toList());
  }

  /**
   * Filter locations by a custom predicate.
   *
   * @param locations List of locations to filter
   * @param filter The filter predicate
   * @return Filtered list
   */
  public static <T extends ScoringLocation> List<T> filterLocations(
      List<T> locations, Predicate<T> filter) {
    return locations.stream().filter(filter).collect(Collectors.toList());
  }

  /**
   * Filter locations by predicate and sort by distance.
   *
   * @param locations List of locations to process
   * @param filter The filter predicate
   * @param fromPose Reference pose for sorting
   * @return Filtered and sorted list
   */
  public static <T extends ScoringLocation> List<T> filterAndSort(
      List<T> locations, Predicate<T> filter, Pose2d fromPose) {
    return locations.stream()
        .filter(filter)
        .sorted(Comparator.comparingDouble(loc -> loc.getDistanceFrom(fromPose)))
        .collect(Collectors.toList());
  }

  /**
   * Simple location database using a map.
   *
   * <p>Game-specific databases can use this pattern:
   *
   * <pre>{@code
   * public class ReefScoringLocations {
   *   private static final Map<String, ReefScoringLocation> LOCATIONS = new HashMap<>();
   *
   *   static {
   *     LOCATIONS.put("E2", new ReefScoringLocation("E2", pose, tagId, level));
   *     // ... more locations
   *   }
   *
   *   public static ReefScoringLocation get(String id) {
   *     return LOCATIONS.get(id);
   *   }
   *
   *   public static List<ReefScoringLocation> getAll() {
   *     return new ArrayList<>(LOCATIONS.values());
   *   }
   * }
   * }</pre>
   *
   * @param <T> The scoring location type
   */
  public static class LocationDatabase<T extends ScoringLocation> {
    protected final Map<String, T> locations = new HashMap<>();

    /** Add a location to the database. */
    public void add(T location) {
      locations.put(location.getId(), location);
    }

    /** Add multiple locations to the database. */
    public void addAll(Collection<T> locations) {
      locations.forEach(this::add);
    }

    /** Get a location by ID. */
    public Optional<T> get(String id) {
      return Optional.ofNullable(locations.get(id));
    }

    /** Get all locations. */
    public List<T> getAll() {
      return new ArrayList<>(locations.values());
    }

    /** Check if a location exists. */
    public boolean contains(String id) {
      return locations.containsKey(id);
    }

    /** Get the number of locations. */
    public int size() {
      return locations.size();
    }

    /** Get all location IDs. */
    public Set<String> getIds() {
      return new HashSet<>(locations.keySet());
    }
  }
}
