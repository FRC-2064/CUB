package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class Cub {

  /** SuperStructure alignment configuration */
  public static class SuperStructureAlignment {
    // Alignment PID gains (X and Y translation)
    public static final double ALIGNMENT_LINEAR_KP = 3.5;
    public static final double ALIGNMENT_LINEAR_KI = 0.0;
    public static final double ALIGNMENT_LINEAR_KD = 0.1;

    // Alignment angular PID gains (rotation during full alignment)
    public static final double ALIGNMENT_ANGULAR_KP = 4.0;
    public static final double ALIGNMENT_ANGULAR_KI = 0.0;
    public static final double ALIGNMENT_ANGULAR_KD = 0.1;

    // Aiming PID gains (rotation-only mode)
    public static final double AIM_ANGULAR_KP = 5.0;
    public static final double AIM_ANGULAR_KI = 0.0;
    public static final double AIM_ANGULAR_KD = 0.2;

    // Motion constraints for alignment
    public static final LinearVelocity MAX_ALIGNMENT_VELOCITY = MetersPerSecond.of(2.5);
    public static final LinearAcceleration MAX_ALIGNMENT_ACCELERATION =
        MetersPerSecondPerSecond.of(3.0);
    public static final AngularVelocity MAX_ALIGNMENT_ANGULAR_VELOCITY = RadiansPerSecond.of(3.0);
    public static final AngularAcceleration MAX_ALIGNMENT_ANGULAR_ACCELERATION =
        RadiansPerSecond.per(Second).of(4.0);

    // Motion constraints for aiming
    public static final AngularVelocity MAX_AIM_ANGULAR_VELOCITY = RadiansPerSecond.of(4.0);
    public static final AngularAcceleration MAX_AIM_ANGULAR_ACCELERATION =
        RadiansPerSecond.per(Second).of(6.0);

    // Tolerances
    public static final Distance POSITION_TOLERANCE = Meters.of(0.05);
    public static final Angle ROTATION_TOLERANCE = Degrees.of(2.0);
    public static final Angle AIM_TOLERANCE = Degrees.of(1.0);

    // Tag tracking
    public static final Time TAG_LOST_TIMEOUT = Seconds.of(0.5);
    public static final AngularVelocity SEARCH_ROTATION_SPEED = RadiansPerSecond.of(1.0);
  }

  /** Pathfinding configuration */
  public static class Pathfinding {
    /** Maximum velocity for pathfinding (meters per second) */
    public static final double MAX_VELOCITY_MPS = 3.0;

    /** Maximum acceleration for pathfinding (meters per second squared) */
    public static final double MAX_ACCELERATION_MPSS = 3.0;

    /** Maximum angular velocity for pathfinding (radians per second) */
    public static final double MAX_ANGULAR_VELOCITY_RPS = Units.degreesToRadians(360);

    /** Maximum angular acceleration for pathfinding (radians per second squared) */
    public static final double MAX_ANGULAR_ACCELERATION_RPSS = Units.degreesToRadians(540);

    /** PathPlanner constraints for autonomous pathfinding */
    public static final PathConstraints CONSTRAINTS =
        new PathConstraints(
            MAX_VELOCITY_MPS,
            MAX_ACCELERATION_MPSS,
            MAX_ANGULAR_VELOCITY_RPS,
            MAX_ANGULAR_ACCELERATION_RPSS);

    /**
     * Distance threshold to switch from pathfinding to vision alignment (meters). When the robot is
     * within this distance of the target AND the target tag is visible, it switches to vision-based
     * alignment.
     */
    public static final double VISION_SWITCH_DISTANCE_METERS = 2.0;

    /**
     * Distance threshold to determine if pathfinding has reached the approach pose (meters). Used
     * to transition from pathfinding to vision alignment phase.
     */
    public static final double PATHFINDING_TOLERANCE_METERS = 0.1;

    /**
     * Rotation threshold to determine if pathfinding has reached the approach pose (radians). Used
     * to transition from pathfinding to vision alignment phase.
     */
    public static final double PATHFINDING_TOLERANCE_RADIANS = Units.degreesToRadians(5.0);
  }

  /** Vision alignment configuration */
  public static class VisionAlignment {
    /** Linear P gain for vision alignment (real robot) */
    public static final double LINEAR_KP = 2.5;

    /** Linear I gain for vision alignment (real robot) */
    public static final double LINEAR_KI = 0.0;

    /** Linear D gain for vision alignment (real robot) */
    public static final double LINEAR_KD = 0.0;

    /** Angular P gain for vision alignment (real robot) */
    public static final double ANGULAR_KP = 2.5;

    /** Angular I gain for vision alignment (real robot) */
    public static final double ANGULAR_KI = 0.0;

    /** Angular D gain for vision alignment (real robot) */
    public static final double ANGULAR_KD = 0.0;

    /** Linear P gain for vision alignment (simulation) */
    public static final double SIM_LINEAR_KP = 2.5;

    /** Linear I gain for vision alignment (simulation) */
    public static final double SIM_LINEAR_KI = 0.0;

    /** Linear D gain for vision alignment (simulation) */
    public static final double SIM_LINEAR_KD = 0.0;

    /** Angular P gain for vision alignment (simulation) */
    public static final double SIM_ANGULAR_KP = 2.5;

    /** Angular I gain for vision alignment (simulation) */
    public static final double SIM_ANGULAR_KI = 0.0;

    /** Angular D gain for vision alignment (simulation) */
    public static final double SIM_ANGULAR_KD = 0.0;

    /** Maximum velocity during vision alignment (meters per second) */
    public static final double MAX_VELOCITY_MPS = 2.0;

    /** Maximum acceleration during vision alignment (meters per second squared) */
    public static final double MAX_ACCELERATION_MPSS = 2.0;

    /** Maximum angular velocity during vision alignment (radians per second) */
    public static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;

    /** Maximum angular acceleration during vision alignment (radians per second squared) */
    public static final double MAX_ANGULAR_ACCELERATION_RPSS = 2.0;

    /** Position tolerance for alignment (meters) */
    public static final double POSITION_TOLERANCE_METERS = 0.1;

    /** Rotation tolerance for alignment (radians) */
    public static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);
  }
}
