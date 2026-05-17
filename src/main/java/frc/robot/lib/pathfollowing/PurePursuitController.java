package frc.robot.lib.pathfollowing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import java.util.List;

/**
 * Core pure pursuit controller implementing adaptive lookahead and velocity profiling.
 */
public class PurePursuitController {
    private final double minLookahead;
    private final double lookaheadVelocityGain;
    private final double maxLookahead;

    private double lastVelocity;
    private static final double LOOP_TIME = 0.02; // 20ms loop time

    /**
     * Creates a pure pursuit controller with configurable lookahead parameters.
     *
     * @param minLookahead Minimum lookahead distance in meters
     * @param lookaheadVelocityGain Lookahead gain per m/s of velocity
     * @param maxLookahead Maximum lookahead distance in meters
     */
    public PurePursuitController(double minLookahead, double lookaheadVelocityGain, double maxLookahead) {
        this.minLookahead = minLookahead;
        this.lookaheadVelocityGain = lookaheadVelocityGain;
        this.maxLookahead = maxLookahead;
        this.lastVelocity = 0.0;
    }

    /**
     * Resets the controller state (call at start of path following).
     */
    public void reset() {
        lastVelocity = 0.0;
    }

    /**
     * Calculates adaptive lookahead distance based on current velocity.
     *
     * @param currentVelocity Current robot velocity in m/s
     * @return Adaptive lookahead distance in meters
     */
    public double calculateAdaptiveLookahead(double currentVelocity) {
        double lookahead = lookaheadVelocityGain * currentVelocity + minLookahead;
        return Math.max(minLookahead, Math.min(maxLookahead, lookahead));
    }

    /**
     * Finds the closest segment index to the robot.
     *
     * @param robotPose Current robot pose
     * @param waypoints List of path waypoints
     * @param currentSegment Current segment index (prevents going backwards)
     * @return Index of closest segment
     */
    public int findClosestSegment(Pose2d robotPose, List<PathWaypoint> waypoints, int currentSegment) {
        if (waypoints.size() < 2) {
            return 0;
        }

        Translation2d robotPos = robotPose.getTranslation();
        int closestSegment = currentSegment;
        double minDistance = Double.POSITIVE_INFINITY;

        // Check current segment and forward segments
        for (int i = currentSegment; i < waypoints.size() - 1; i++) {
            Translation2d segmentStart = waypoints.get(i).getTranslation();
            Translation2d segmentEnd = waypoints.get(i + 1).getTranslation();

            double distance = distanceToSegment(robotPos, segmentStart, segmentEnd);
            if (distance < minDistance) {
                minDistance = distance;
                closestSegment = i;
            }
        }

        return closestSegment;
    }

    /**
     * Finds the lookahead point on the path.
     *
     * @param robotPose Current robot pose
     * @param waypoints List of path waypoints
     * @param currentSegment Current segment index
     * @param lookaheadDistance Lookahead distance
     * @return Lookahead point, or last waypoint if no intersection found
     */
    public Translation2d calculateLookaheadPoint(
            Pose2d robotPose,
            List<PathWaypoint> waypoints,
            int currentSegment,
            double lookaheadDistance) {

        Translation2d robotPos = robotPose.getTranslation();

        // Get path direction at robot's location for validation
        Translation2d pathDirection = getPathDirection(robotPos, waypoints, currentSegment);

        Translation2d bestIntersection = null;
        double maxProgressAlongPath = -1.0;

        // Search from current segment forward
        for (int i = currentSegment; i < waypoints.size() - 1; i++) {
            Translation2d segmentStart = waypoints.get(i).getTranslation();
            Translation2d segmentEnd = waypoints.get(i + 1).getTranslation();

            Translation2d intersection = circleSegmentIntersection(
                robotPos, lookaheadDistance, segmentStart, segmentEnd);

            if (intersection != null) {
                // Validate intersection is ahead using dot product with path direction
                Translation2d toIntersection = intersection.minus(robotPos);
                double dotProduct = toIntersection.getX() * pathDirection.getX()
                                  + toIntersection.getY() * pathDirection.getY();

                // Skip intersections that are behind the robot
                if (dotProduct < 0.1) {  // Small threshold to avoid numerical issues
                    continue;
                }

                // Calculate progress along path (segment index + position on segment)
                Translation2d segmentVec = segmentEnd.minus(segmentStart);
                Translation2d toIntersectionFromStart = intersection.minus(segmentStart);
                double segmentLength = segmentVec.getNorm();
                double progressOnSegment = 0.0;

                if (segmentLength > 0.001) {
                    double projection = (toIntersectionFromStart.getX() * segmentVec.getX()
                                       + toIntersectionFromStart.getY() * segmentVec.getY())
                                       / (segmentLength * segmentLength);
                    progressOnSegment = Math.max(0, Math.min(1, projection));
                }

                double totalProgress = i + progressOnSegment;

                // Keep furthest intersection along path
                if (totalProgress > maxProgressAlongPath) {
                    maxProgressAlongPath = totalProgress;
                    bestIntersection = intersection;
                }
            }
        }

        // If no valid intersection found, project forward along path
        if (bestIntersection == null) {
            bestIntersection = projectAlongPath(robotPos, waypoints, currentSegment, lookaheadDistance);
        }

        return bestIntersection;
    }

    /**
     * Gets the path direction at the robot's current location.
     *
     * @param robotPos Robot position
     * @param waypoints Path waypoints
     * @param currentSegment Current segment index
     * @return Normalized direction vector along the path
     */
    private Translation2d getPathDirection(
            Translation2d robotPos,
            List<PathWaypoint> waypoints,
            int currentSegment) {

        if (currentSegment >= waypoints.size() - 1) {
            // At end of path, use direction of last segment
            Translation2d start = waypoints.get(waypoints.size() - 2).getTranslation();
            Translation2d end = waypoints.get(waypoints.size() - 1).getTranslation();
            Translation2d dir = end.minus(start);
            double length = dir.getNorm();
            return length > 0.001 ? dir.div(length) : new Translation2d(1, 0);
        }

        Translation2d segmentStart = waypoints.get(currentSegment).getTranslation();
        Translation2d segmentEnd = waypoints.get(currentSegment + 1).getTranslation();
        Translation2d direction = segmentEnd.minus(segmentStart);
        double length = direction.getNorm();

        // Return normalized direction (or default forward if segment is too small)
        return length > 0.001 ? direction.div(length) : new Translation2d(1, 0);
    }

    /**
     * Projects forward along the path from the robot's position by a given distance.
     * Used as fallback when no circle intersection is found.
     *
     * @param robotPos Robot position
     * @param waypoints Path waypoints
     * @param currentSegment Current segment index
     * @param distance Distance to project forward
     * @return Projected point on path
     */
    private Translation2d projectAlongPath(
            Translation2d robotPos,
            List<PathWaypoint> waypoints,
            int currentSegment,
            double distance) {

        // Start from robot's position on current segment
        int segment = currentSegment;
        double remainingDistance = distance;

        // Find closest point on current segment to robot
        Translation2d segmentStart = waypoints.get(segment).getTranslation();
        Translation2d segmentEnd = waypoints.get(Math.min(segment + 1, waypoints.size() - 1)).getTranslation();

        Translation2d currentPos = closestPointOnSegment(robotPos, segmentStart, segmentEnd);

        // Walk forward along path
        while (remainingDistance > 0 && segment < waypoints.size() - 1) {
            segmentStart = waypoints.get(segment).getTranslation();
            segmentEnd = waypoints.get(segment + 1).getTranslation();

            double distToSegmentEnd = currentPos.getDistance(segmentEnd);

            if (distToSegmentEnd <= remainingDistance) {
                // Move to next segment
                remainingDistance -= distToSegmentEnd;
                currentPos = segmentEnd;
                segment++;
            } else {
                // Target is on this segment
                Translation2d direction = segmentEnd.minus(currentPos);
                double segmentLength = direction.getNorm();
                if (segmentLength > 0.001) {
                    direction = direction.div(segmentLength);
                    currentPos = currentPos.plus(direction.times(remainingDistance));
                }
                break;
            }
        }

        return currentPos;
    }

    /**
     * Finds the closest point on a line segment to a given point.
     *
     * @param point The point
     * @param segmentStart Segment start
     * @param segmentEnd Segment end
     * @return Closest point on segment
     */
    private Translation2d closestPointOnSegment(
            Translation2d point,
            Translation2d segmentStart,
            Translation2d segmentEnd) {

        Translation2d segment = segmentEnd.minus(segmentStart);
        Translation2d toPoint = point.minus(segmentStart);

        double segmentLengthSquared = segment.getX() * segment.getX()
            + segment.getY() * segment.getY();

        if (segmentLengthSquared < 0.0001) {
            return segmentStart;
        }

        double t = (toPoint.getX() * segment.getX() + toPoint.getY() * segment.getY())
            / segmentLengthSquared;
        t = Math.max(0, Math.min(1, t));

        return segmentStart.plus(segment.times(t));
    }

    /**
     * Checks if robot should advance to next segment based on handoff radius.
     *
     * @param robotPose Current robot pose
     * @param waypoint The waypoint to check
     * @return True if robot is within handoff radius
     */
    public boolean shouldAdvanceSegment(Pose2d robotPose, PathWaypoint waypoint) {
        double distance = robotPose.getTranslation().getDistance(waypoint.getTranslation());
        return distance < waypoint.getHandoffRadius();
    }

    /**
     * Calculates target velocity using trapezoidal profile.
     *
     * @param path The parsed path
     * @param currentSegment Current segment index
     * @param robotPose Current robot pose
     * @param distanceToEnd Distance remaining to path end
     * @return Target velocity in m/s
     */
    public double calculateTargetVelocity(
            ParsedPath path,
            int currentSegment,
            Pose2d robotPose,
            double distanceToEnd) {

        // Get constraints for current segment
        double maxVelocity = path.getMaxVelocity(currentSegment);
        double maxAcceleration = path.getMaxAcceleration(currentSegment);

        // Velocity limit based on remaining distance
        double velocityForDistance = Math.sqrt(2.0 * maxAcceleration * distanceToEnd);

        // Don't exceed what we can stop in
        double targetVel = Math.min(maxVelocity, velocityForDistance);

        // Apply acceleration limits (trapezoidal profile)
        double maxVelocityIncrease = lastVelocity + maxAcceleration * LOOP_TIME;
        double maxVelocityDecrease = lastVelocity - maxAcceleration * LOOP_TIME;

        if (targetVel > lastVelocity) {
            // Accelerating
            targetVel = Math.min(targetVel, maxVelocityIncrease);
        } else {
            // Decelerating
            targetVel = Math.max(targetVel, maxVelocityDecrease);
        }

        // Only decelerate near the final waypoint (pure pursuit handles intermediate waypoints naturally)
        if (currentSegment >= path.size() - 2) {
            PathWaypoint finalWaypoint = path.getWaypoint(path.size() - 1);
            double distanceToFinal = robotPose.getTranslation()
                .getDistance(finalWaypoint.getTranslation());

            // Use the path's end translation tolerance as the deceleration radius
            double decelerationRadius = path.getEndTranslationTolerance()
                * Constants.PurePursuitConstants.WAYPOINT_DECEL_RADIUS_MULT;

            if (distanceToFinal < decelerationRadius) {
                double decelerationFactor = distanceToFinal / decelerationRadius;
                targetVel *= Math.max(0.5, decelerationFactor); // Don't go below 50% speed
            }
        }

        // Enforce minimum velocity
        targetVel = Math.max(Constants.PurePursuitConstants.MIN_VELOCITY, targetVel);

        lastVelocity = targetVel;
        return targetVel;
    }

    /**
     * Calculates field-relative translation velocity toward lookahead point.
     *
     * @param robotPose Current robot pose
     * @param lookaheadPoint The lookahead point
     * @param targetVelocity Target velocity magnitude
     * @return Field-relative translation velocity
     */
    public Translation2d calculateTranslationVelocity(
            Pose2d robotPose,
            Translation2d lookaheadPoint,
            double targetVelocity) {

        // Vector from robot to lookahead point
        Translation2d toTarget = lookaheadPoint.minus(robotPose.getTranslation());

        // Normalize and scale by target velocity
        double distance = toTarget.getNorm();
        if (distance < 0.001) {
            return new Translation2d();
        }

        return toTarget.div(distance).times(targetVelocity);
    }

    /**
     * Calculates remaining distance to path end.
     *
     * @param robotPose Current robot pose
     * @param waypoints List of path waypoints
     * @param currentSegment Current segment index
     * @return Remaining distance in meters
     */
    public double calculateDistanceToEnd(
            Pose2d robotPose,
            List<PathWaypoint> waypoints,
            int currentSegment) {

        if (waypoints.isEmpty()) {
            return 0.0;
        }

        Translation2d robotPos = robotPose.getTranslation();
        Translation2d finalWaypoint = waypoints.get(waypoints.size() - 1).getTranslation();

        // If we're at or past the last segment, just return distance to final waypoint
        if (currentSegment >= waypoints.size() - 1) {
            return robotPos.getDistance(finalWaypoint);
        }

        double totalDistance = 0.0;

        // Distance from robot to end of current segment
        Translation2d segmentEnd = waypoints.get(currentSegment + 1).getTranslation();
        totalDistance += robotPos.getDistance(segmentEnd);

        // Add remaining segment lengths
        for (int i = currentSegment + 1; i < waypoints.size() - 1; i++) {
            Translation2d start = waypoints.get(i).getTranslation();
            Translation2d end = waypoints.get(i + 1).getTranslation();
            totalDistance += start.getDistance(end);
        }

        return totalDistance;
    }

    /**
     * Finds intersection of circle with line segment (furthest point).
     *
     * @param center Circle center
     * @param radius Circle radius
     * @param segmentStart Line segment start
     * @param segmentEnd Line segment end
     * @return Intersection point, or null if no intersection
     */
    private Translation2d circleSegmentIntersection(
            Translation2d center,
            double radius,
            Translation2d segmentStart,
            Translation2d segmentEnd) {

        // Vector from start to end
        Translation2d d = segmentEnd.minus(segmentStart);
        // Vector from start to circle center
        Translation2d f = segmentStart.minus(center);

        double a = d.getX() * d.getX() + d.getY() * d.getY();
        double b = 2 * (f.getX() * d.getX() + f.getY() * d.getY());
        double c = f.getX() * f.getX() + f.getY() * f.getY() - radius * radius;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // No intersection
            return null;
        }

        discriminant = Math.sqrt(discriminant);

        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        // Check if intersections are on the segment (t in [0, 1])
        Translation2d intersection = null;

        if (t2 >= 0 && t2 <= 1) {
            // Use furthest intersection (t2)
            intersection = segmentStart.plus(d.times(t2));
        } else if (t1 >= 0 && t1 <= 1) {
            // Use closer intersection (t1)
            intersection = segmentStart.plus(d.times(t1));
        }

        return intersection;
    }

    /**
     * Calculates distance from point to line segment.
     *
     * @param point The point
     * @param segmentStart Line segment start
     * @param segmentEnd Line segment end
     * @return Distance in meters
     */
    private double distanceToSegment(
            Translation2d point,
            Translation2d segmentStart,
            Translation2d segmentEnd) {

        Translation2d segment = segmentEnd.minus(segmentStart);
        Translation2d toPoint = point.minus(segmentStart);

        double segmentLengthSquared = segment.getX() * segment.getX()
            + segment.getY() * segment.getY();

        if (segmentLengthSquared < 0.0001) {
            // Segment is essentially a point
            return point.getDistance(segmentStart);
        }

        // Project point onto segment
        double t = (toPoint.getX() * segment.getX() + toPoint.getY() * segment.getY())
            / segmentLengthSquared;
        t = Math.max(0, Math.min(1, t)); // Clamp to [0, 1]

        Translation2d projection = segmentStart.plus(segment.times(t));
        return point.getDistance(projection);
    }
}
