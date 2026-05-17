package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.pathfollowing.BLinePathLoader;
import frc.robot.lib.pathfollowing.ParsedPath;
import frc.robot.lib.pathfollowing.PathWaypoint;
import frc.robot.lib.pathfollowing.PurePursuitController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Command that follows a path using adaptive pure pursuit for translation
 * and existing YAGSL heading control for rotation.
 */
public class AdaptivePurePursuitCommand extends Command {
    private final SwerveSubsystem drivebase;
    private final String pathName;
    private final boolean shouldFlip;
    private final boolean shouldMirror;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> velocitySupplier;
    private final Consumer<Pose2d> resetOdometry;
    private final PurePursuitController controller;
    private final PIDController headingController;

    private ParsedPath path;  // Loaded at runtime in initialize()
    private int currentSegment;
    private boolean isFinished;

    /**
     * Creates a new adaptive pure pursuit command.
     *
     * @param drivebase The swerve drive subsystem
     * @param pathName The name of the path to follow
     * @param shouldFlip Whether to flip for red alliance
     * @param shouldMirror Whether to mirror across Y-axis center
     * @param poseSupplier Supplier for current robot pose
     * @param velocitySupplier Supplier for current field-relative velocity
     * @param resetOdometry Consumer to reset odometry (null if no reset)
     * @param controller The pure pursuit controller instance
     */
    public AdaptivePurePursuitCommand(
            SwerveSubsystem drivebase,
            String pathName,
            boolean shouldFlip,
            boolean shouldMirror,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> velocitySupplier,
            Consumer<Pose2d> resetOdometry,
            PurePursuitController controller) {
        this.drivebase = drivebase;
        this.pathName = pathName;
        this.shouldFlip = shouldFlip;
        this.shouldMirror = shouldMirror;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
        this.resetOdometry = resetOdometry;
        this.controller = controller;

        // Heading PID controller (matching BLine's rotation PID: kP=3.0)
        this.headingController = new PIDController(3.0, 0.0, 0.0);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        currentSegment = 0;
        isFinished = false;
        controller.reset();
        headingController.reset();

        // Check alliance at runtime and load path with appropriate flipping
        boolean flipPath = shouldFlip && isRedAlliance();
        System.out.println("AdaptivePurePursuitCommand: Loading path '" + pathName
            + "' (alliance=" + (isRedAlliance() ? "RED" : "BLUE")
            + ", flip=" + flipPath + ", mirror=" + shouldMirror + ")");

        // Load the path at runtime (defers alliance check until now)
        path = BLinePathLoader.loadPath(pathName, flipPath);

        // Mirror the path across Y-axis center if requested (for left/right symmetry)
        if (shouldMirror) {
            path = BLinePathLoader.mirrorPath(path);
        }

        if (path.isEmpty()) {
            System.err.println("AdaptivePurePursuitCommand: ERROR - Path is empty!");
            isFinished = true;
            return;
        }

        // Reset odometry to path start if configured
        // Only reset if no vision target available OR in simulation mode
        if (resetOdometry != null) {
            boolean hasVisionTarget = drivebase.hasActiveVisionTarget();
            boolean isSimulation = RobotBase.isSimulation();

            if (!hasVisionTarget || isSimulation) {
                Pose2d startPose = path.getStartPose();
                resetOdometry.accept(startPose);
                System.out.println("AdaptivePurePursuitCommand: Reset odometry to "
                    + String.format("(%.2f, %.2f, %.1f°)",
                        startPose.getX(), startPose.getY(),
                        Math.toDegrees(startPose.getRotation().getRadians()))
                    + " (vision=" + (hasVisionTarget ? "available" : "none")
                    + ", sim=" + isSimulation + ")");
            } else {
                System.out.println("AdaptivePurePursuitCommand: Skipping odometry reset - vision target available");
            }
        }

        System.out.println("AdaptivePurePursuitCommand: Starting path with "
            + path.size() + " waypoints, " + (path.size() - 1) + " segments");
    }

    /**
     * Checks if the robot is on red alliance.
     *
     * @return True if red alliance, false otherwise
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    @Override
    public void execute() {
        if (path.isEmpty()) {
            isFinished = true;
            return;
        }

        // Clamp segment to valid range (segments are 0 to size-2)
        currentSegment = Math.max(0, Math.min(currentSegment, path.size() - 2));

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentVelocity = velocitySupplier.get();

        // Calculate current speed
        double currentSpeed = Math.hypot(
            currentVelocity.vxMetersPerSecond,
            currentVelocity.vyMetersPerSecond
        );

        // Find closest segment (but don't go backwards, and stay within valid segment range)
        int closestSegment = controller.findClosestSegment(
            currentPose, path.getWaypoints(), currentSegment);

        // Only advance if the closest segment is within one step forward
        // This prevents jumping to the end of the path
        if (closestSegment <= currentSegment + 1 && closestSegment < path.size() - 1) {
            if (closestSegment > currentSegment) {
                currentSegment = closestSegment;
                System.out.println("Advanced to segment " + currentSegment + "/" + (path.size() - 1));
            }
        }

        // Additional check: advance based on handoff radius
        if (currentSegment < path.size() - 1) {
            PathWaypoint nextWaypoint = path.getWaypoint(currentSegment + 1);
            if (controller.shouldAdvanceSegment(currentPose, nextWaypoint)) {
                currentSegment++;
                System.out.println("Handoff advance to segment " + currentSegment + "/" + (path.size() - 1));
            }
        }

        // Calculate adaptive lookahead
        double lookaheadDistance = controller.calculateAdaptiveLookahead(currentSpeed);

        // Find lookahead point
        Translation2d lookaheadPoint = controller.calculateLookaheadPoint(
            currentPose, path.getWaypoints(), currentSegment, lookaheadDistance);

        // Calculate distance to end
        double distanceToEnd = controller.calculateDistanceToEnd(
            currentPose, path.getWaypoints(), currentSegment);

        // Calculate target velocity using trapezoidal profile
        double targetVelocity = controller.calculateTargetVelocity(
            path, currentSegment, currentPose, distanceToEnd);

        // Calculate translation velocity toward lookahead point (field-relative)
        Translation2d translationVelocity = controller.calculateTranslationVelocity(
            currentPose, lookaheadPoint, targetVelocity);

        // Get target heading from current waypoint
        PathWaypoint currentWaypoint = path.getWaypoint(
            Math.min(currentSegment + 1, path.size() - 1));
        Rotation2d targetHeading = currentWaypoint.getRotation();
        Translation2d targetWaypointPos = currentWaypoint.getTranslation();

        // Calculate heading error and use PID to get omega (in radians)
        double headingError = targetHeading.getRadians() - currentPose.getRotation().getRadians();
        // Normalize heading error to [-π, π] for display
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        double omega = headingController.calculate(currentPose.getRotation().getRadians(),
                                                   targetHeading.getRadians());

        // Clamp omega to reasonable limits (prevent extreme rotation commands)
        // Max rotation speed: ~360 deg/sec = 2π rad/sec
        final double MAX_OMEGA = 2 * Math.PI;
        omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, omega));

        // Create field-relative ChassisSpeeds directly (no joystick cubing)
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(
            translationVelocity.getX(),
            translationVelocity.getY(),
            omega
        );

        // Drive the robot with field-oriented control
        drivebase.driveFieldOriented(targetSpeeds);

        // SmartDashboard telemetry
        SmartDashboard.putNumber("PP/CurrentSegment", currentSegment);
        SmartDashboard.putNumber("PP/TotalSegments", path.size() - 1);
        SmartDashboard.putNumber("PP/RobotX", currentPose.getX());
        SmartDashboard.putNumber("PP/RobotY", currentPose.getY());
        SmartDashboard.putNumber("PP/RobotHeading", Math.toDegrees(currentPose.getRotation().getRadians()));
        SmartDashboard.putNumber("PP/TargetWaypointX", targetWaypointPos.getX());
        SmartDashboard.putNumber("PP/TargetWaypointY", targetWaypointPos.getY());
        SmartDashboard.putNumber("PP/CurrentHeadingRaw", currentPose.getRotation().getRadians());
        SmartDashboard.putNumber("PP/TargetHeadingRaw", targetHeading.getRadians());
        SmartDashboard.putNumber("PP/LookaheadX", lookaheadPoint.getX());
        SmartDashboard.putNumber("PP/LookaheadY", lookaheadPoint.getY());
        SmartDashboard.putNumber("PP/LookaheadDist", lookaheadDistance);
        SmartDashboard.putNumber("PP/DistanceToEnd", distanceToEnd);
        SmartDashboard.putNumber("PP/CurrentSpeed", currentSpeed);
        SmartDashboard.putNumber("PP/TargetVelocity", targetVelocity);
        SmartDashboard.putNumber("PP/TargetHeading", Math.toDegrees(targetHeading.getRadians()));
        SmartDashboard.putNumber("PP/HeadingError", Math.toDegrees(headingError));
        SmartDashboard.putNumber("PP/Omega", omega);
        SmartDashboard.putNumber("PP/TransVelX", translationVelocity.getX());
        SmartDashboard.putNumber("PP/TransVelY", translationVelocity.getY());

        // Check for path completion
        if (currentSegment >= path.size() - 1) {
            checkPathCompletion(currentPose, currentSpeed);
        }
    }

    /**
     * Checks if the path is complete based on tolerances.
     *
     * @param currentPose Current robot pose
     * @param currentSpeed Current robot speed
     */
    private void checkPathCompletion(Pose2d currentPose, double currentSpeed) {
        PathWaypoint finalWaypoint = path.getWaypoint(path.size() - 1);
        double distanceToEnd = currentPose.getTranslation()
            .getDistance(finalWaypoint.getTranslation());

        boolean positionComplete = distanceToEnd < path.getEndTranslationTolerance();
        boolean velocityComplete = currentSpeed < path.getEndVelocityTolerance();

        if (positionComplete && velocityComplete) {
            isFinished = true;
            System.out.println("AdaptivePurePursuitCommand: Path complete!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(), 0, true);

        if (interrupted) {
            System.out.println("AdaptivePurePursuitCommand: Interrupted");
        } else {
            System.out.println("AdaptivePurePursuitCommand: Completed normally");
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
