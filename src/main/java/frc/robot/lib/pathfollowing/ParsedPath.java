package frc.robot.lib.pathfollowing;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/**
 * Internal representation of a parsed BLine path.
 * Contains waypoints, velocity/acceleration constraints per segment, and completion tolerances.
 */
public class ParsedPath {
    private final List<PathWaypoint> waypoints;
    private final double[] maxVelocityPerSegment;
    private final double[] maxAccelerationPerSegment;
    private final double endTranslationTolerance;
    private final double endRotationTolerance;
    private final double endVelocityTolerance;

    /**
     * Creates a parsed path with all constraints.
     *
     * @param waypoints List of waypoints in the path
     * @param maxVelocityPerSegment Maximum velocity for each segment in m/s
     * @param maxAccelerationPerSegment Maximum acceleration for each segment in m/s^2
     * @param endTranslationTolerance Position tolerance for completion in meters
     * @param endRotationTolerance Rotation tolerance for completion in degrees
     * @param endVelocityTolerance Velocity tolerance for completion in m/s
     */
    public ParsedPath(
            List<PathWaypoint> waypoints,
            double[] maxVelocityPerSegment,
            double[] maxAccelerationPerSegment,
            double endTranslationTolerance,
            double endRotationTolerance,
            double endVelocityTolerance) {
        this.waypoints = waypoints;
        this.maxVelocityPerSegment = maxVelocityPerSegment;
        this.maxAccelerationPerSegment = maxAccelerationPerSegment;
        this.endTranslationTolerance = endTranslationTolerance;
        this.endRotationTolerance = endRotationTolerance;
        this.endVelocityTolerance = endVelocityTolerance;
    }

    /**
     * Gets all waypoints in the path.
     *
     * @return List of waypoints
     */
    public List<PathWaypoint> getWaypoints() {
        return waypoints;
    }

    /**
     * Gets a specific waypoint by index.
     *
     * @param index The waypoint index (0-based)
     * @return The waypoint at the specified index
     */
    public PathWaypoint getWaypoint(int index) {
        return waypoints.get(index);
    }

    /**
     * Gets the maximum velocity for a specific segment.
     *
     * @param segmentIndex The segment index (0-based)
     * @return Maximum velocity in m/s
     */
    public double getMaxVelocity(int segmentIndex) {
        if (segmentIndex < 0 || segmentIndex >= maxVelocityPerSegment.length) {
            return maxVelocityPerSegment[maxVelocityPerSegment.length - 1];
        }
        return maxVelocityPerSegment[segmentIndex];
    }

    /**
     * Gets the maximum acceleration for a specific segment.
     *
     * @param segmentIndex The segment index (0-based)
     * @return Maximum acceleration in m/s^2
     */
    public double getMaxAcceleration(int segmentIndex) {
        if (segmentIndex < 0 || segmentIndex >= maxAccelerationPerSegment.length) {
            return maxAccelerationPerSegment[maxAccelerationPerSegment.length - 1];
        }
        return maxAccelerationPerSegment[segmentIndex];
    }

    /**
     * Gets the starting pose of the path (first waypoint position and rotation).
     *
     * @return The starting pose
     */
    public Pose2d getStartPose() {
        if (waypoints.isEmpty()) {
            return new Pose2d();
        }
        PathWaypoint first = waypoints.get(0);
        return new Pose2d(first.getTranslation(), first.getRotation());
    }

    /**
     * Gets the end translation tolerance for path completion.
     *
     * @return Position tolerance in meters
     */
    public double getEndTranslationTolerance() {
        return endTranslationTolerance;
    }

    /**
     * Gets the end rotation tolerance for path completion.
     *
     * @return Rotation tolerance in degrees
     */
    public double getEndRotationTolerance() {
        return endRotationTolerance;
    }

    /**
     * Gets the end velocity tolerance for path completion.
     *
     * @return Velocity tolerance in m/s
     */
    public double getEndVelocityTolerance() {
        return endVelocityTolerance;
    }

    /**
     * Gets the velocity constraint array.
     *
     * @return Array of max velocities per segment
     */
    public double[] getMaxVelocityPerSegment() {
        return maxVelocityPerSegment;
    }

    /**
     * Gets the acceleration constraint array.
     *
     * @return Array of max accelerations per segment
     */
    public double[] getMaxAccelerationPerSegment() {
        return maxAccelerationPerSegment;
    }

    /**
     * Gets the number of waypoints in the path.
     *
     * @return Number of waypoints
     */
    public int size() {
        return waypoints.size();
    }

    /**
     * Checks if the path is empty.
     *
     * @return True if path has no waypoints
     */
    public boolean isEmpty() {
        return waypoints.isEmpty();
    }

    @Override
    public String toString() {
        return String.format("ParsedPath(waypoints=%d, start=%s)",
            waypoints.size(),
            waypoints.isEmpty() ? "empty" : waypoints.get(0).toString());
    }
}
