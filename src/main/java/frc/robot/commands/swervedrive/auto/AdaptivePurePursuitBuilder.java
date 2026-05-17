package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.pathfollowing.BLinePath;
import frc.robot.lib.pathfollowing.BLinePathLoader;
import frc.robot.lib.pathfollowing.ParsedPath;
import frc.robot.lib.pathfollowing.PurePursuitController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Builder for creating adaptive pure pursuit path following commands.
 * Provides a drop-in replacement API for BLine's FollowPath.Builder.
 */
public class AdaptivePurePursuitBuilder {
    private final SwerveSubsystem drivebase;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> velocitySupplier;

    private double minLookahead = Constants.PurePursuitConstants.MIN_LOOKAHEAD_DISTANCE;
    private double lookaheadVelocityGain = Constants.PurePursuitConstants.LOOKAHEAD_VELOCITY_GAIN;
    private double maxLookahead = Constants.PurePursuitConstants.MAX_LOOKAHEAD_DISTANCE;

    private boolean shouldFlip = false;
    private boolean shouldMirror = false;
    private Consumer<Pose2d> resetOdometry = null;

    /**
     * Creates a new builder with required dependencies.
     *
     * @param drivebase The swerve drive subsystem
     * @param poseSupplier Supplier for current robot pose
     * @param velocitySupplier Supplier for current field-relative velocity
     */
    public AdaptivePurePursuitBuilder(
            SwerveSubsystem drivebase,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> velocitySupplier) {
        this.drivebase = drivebase;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
    }

    /**
     * Configures lookahead parameters for adaptive pure pursuit.
     *
     * @param minLookahead Minimum lookahead distance in meters
     * @param velocityGain Lookahead distance gain per m/s velocity
     * @param maxLookahead Maximum lookahead distance in meters
     * @return This builder for chaining
     */
    public AdaptivePurePursuitBuilder withLookaheadConfig(
            double minLookahead,
            double velocityGain,
            double maxLookahead) {
        this.minLookahead = minLookahead;
        this.lookaheadVelocityGain = velocityGain;
        this.maxLookahead = maxLookahead;
        return this;
    }

    /**
     * Enables automatic path flipping based on alliance color.
     * Path will be flipped if alliance is red.
     *
     * @return This builder for chaining
     */
    public AdaptivePurePursuitBuilder withDefaultShouldFlip() {
        this.shouldFlip = true;
        return this;
    }

    /**
     * Configures pose reset at path start.
     *
     * @param resetConsumer Consumer that resets odometry to given pose
     * @return This builder for chaining
     */
    public AdaptivePurePursuitBuilder withPoseReset(Consumer<Pose2d> resetConsumer) {
        this.resetOdometry = resetConsumer;
        return this;
    }

    /**
     * Enables Y-axis mirroring of the path (for left/right symmetry).
     * Path will be mirrored across the field center vertically.
     *
     * @return This builder for chaining
     */
    public AdaptivePurePursuitBuilder withMirror() {
        this.shouldMirror = true;
        return this;
    }

    /**
     * Builds a command that follows the specified path.
     * Path loading and alliance checking is deferred until command initialize().
     *
     * @param path The path to follow (contains path name)
     * @return Command that follows the path using adaptive pure pursuit
     */
    public Command build(BLinePath path) {
        // Create pure pursuit controller
        PurePursuitController controller = new PurePursuitController(
            minLookahead, lookaheadVelocityGain, maxLookahead);

        // Create and return command (path loading deferred to initialize())
        return new AdaptivePurePursuitCommand(
            drivebase,
            path.getPathName(),
            shouldFlip,
            shouldMirror,
            poseSupplier,
            velocitySupplier,
            resetOdometry,
            controller
        );
    }
}
