package frc.robot.lib.pathfollowing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a single waypoint in a path.
 * Waypoints contain translation (x, y position), optional rotation target, and handoff radius.
 */
public class PathWaypoint {
    private final Translation2d translation;
    private Rotation2d rotation;
    private final double handoffRadius;
    private final boolean hasRotationTarget;

    /**
     * Creates a waypoint with translation, rotation, and handoff radius.
     *
     * @param translation The x, y position of the waypoint in meters
     * @param rotation The target heading at this waypoint (null if no rotation target)
     * @param handoffRadius The radius in meters at which to transition to the next waypoint
     * @param hasRotationTarget Whether this waypoint originally had a rotation target
     */
    public PathWaypoint(Translation2d translation, Rotation2d rotation, double handoffRadius, boolean hasRotationTarget) {
        this.translation = translation;
        this.rotation = rotation;
        this.handoffRadius = handoffRadius;
        this.hasRotationTarget = hasRotationTarget;
    }

    /**
     * Gets the translation (x, y position) of this waypoint.
     *
     * @return The waypoint position
     */
    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * Gets the target rotation at this waypoint.
     *
     * @return The target heading (may be filled in from previous waypoint for translation-only elements)
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * Sets the rotation for this waypoint (used when filling in rotation for translation-only elements).
     *
     * @param rotation The rotation to set
     */
    public void setRotation(Rotation2d rotation) {
        this.rotation = rotation;
    }

    /**
     * Gets the handoff radius for this waypoint.
     * When the robot gets within this radius, it transitions to tracking the next waypoint.
     *
     * @return The handoff radius in meters
     */
    public double getHandoffRadius() {
        return handoffRadius;
    }

    /**
     * Checks if this waypoint originally had a rotation target in the path JSON.
     *
     * @return True if this was a "waypoint" type element with rotation_target, false for "translation" type
     */
    public boolean hasRotationTarget() {
        return hasRotationTarget;
    }

    @Override
    public String toString() {
        return String.format("PathWaypoint(x=%.2f, y=%.2f, rot=%.2f, handoff=%.2f, hasRot=%b)",
            translation.getX(), translation.getY(),
            rotation != null ? Math.toDegrees(rotation.getRadians()) : 0.0,
            handoffRadius, hasRotationTarget);
    }
}
