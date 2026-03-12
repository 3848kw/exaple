package frc.robot.vision.odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.FieldConstants;
import frc.robot.utils.LimelightHelpers;

/**
 * {@code LimelightOdometryCamera} provides an {@link OdometryCamera} implementation
 * backed by a Limelight running AprilTag pose estimation.
 * <p>
 * This class fetches robot pose estimates from the Limelight (either standard
 * WPI Blue or MegaTag2 pipeline), converts them to {@link PoseEstimate} for
 * downstream fusion, and publishes useful telemetry to NetworkTables:
 * <ul>
 *     <li>Estimated field pose (Pose2d)</li>
 *     <li>Tracked tag poses (Pose3d[] from the loaded field layout)</li>
 *     <li>MegaTag2 enabled flag</li>
 * </ul>
 * Disable conditions can be registered to prevent pose updates from being consumed
 * by the rest of the system, though the raw telemetry can still be published.
 */
public class LimelightOdometryCamera implements OdometryCamera {
    /** A set of conditions that, when true, should disable consuming this camera's pose updates. */
    private List<BooleanSupplier> disableConditions;

    /** Publishes the most recent estimated field pose to NetworkTables. */
    private final StructPublisher<Pose2d> posePublisher;

    /** Publishes the current set of visible/used tag field poses (from the field layout). */
    private final StructArrayPublisher<Pose3d> trackedTargetsPublisher;

    /** Publishes whether MegaTag2 processing is enabled for this camera. */
    private final BooleanPublisher megaTag2Publisher;

    /** Limelight name as configured on the device and used by {@link LimelightHelpers}. */
    private final String name;

    /** Flag controlling whether MegaTag2 mode is used for pose estimation. */
    private boolean megaTag2;

    /**
     * Creates a new {@code LimelightOdometryCamera} with full configuration.
     *
     * @param name              The Limelight name (must match Limelight device configuration).
     * @param megaTag2          If {@code true}, MegaTag2 mode is considered enabled for this camera.
     * @param disableConditions Initial list of conditions that can disable consumption of pose updates.
     */
    public LimelightOdometryCamera(String name, boolean megaTag2, List<BooleanSupplier> disableConditions) {
        this.name = name;
        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/" + name + "/Estimated Pose", Pose2d.struct).publish();
        trackedTargetsPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Vision/" + name + "/Tracked Targets", Pose3d.struct).publish();
        megaTag2Publisher = NetworkTableInstance.getDefault()
                .getBooleanTopic("Vision/" + name + "/MegaTag2").publish();
    }

    /**
     * Convenience constructor that defaults {@code disableConditions} to an empty list.
     *
     * @param name     The Limelight name.
     * @param megaTag2 If {@code true}, MegaTag2 mode is considered enabled.
     */
    public LimelightOdometryCamera(String name, boolean megaTag2) {
        this(name, megaTag2, new ArrayList<>());
    }

    /**
     * Convenience constructor that disables MegaTag2 and uses an empty disable list.
     *
     * @param name The Limelight name.
     */
    public LimelightOdometryCamera(String name) {
        this(name, false, new ArrayList<>());
    }

    /**
     * Polls the Limelight for the latest pose estimate and returns it if available.
     * <p>
     * Behavior:
     * <ul>
     *   <li>Reads a pose estimate from Limelight via {@link LimelightHelpers}.</li>
     *   <li>Publishes the MegaTag2 enable flag to NetworkTables.</li>
     *   <li>If a pose is present, wraps it in a {@link PoseEstimate} with a fixed
     *       process noise of (0.5, 0.5, 0.5) for (x, y, theta) and publishes
     *       telemetry for the pose and currently tracked tag field poses.</li>
     *   <li>Returns {@link Optional#empty()} when no pose is available.</li>
     * </ul>
     *
     * @return An {@link Optional} containing the latest {@link PoseEstimate}, or empty if no pose is available.
     */
    @Override
    public Optional<PoseEstimate> update() {
        Optional<PoseEstimate> estimatedPose = Optional.empty();
        // Note: The choice of helper here is controlled by the 'megaTag2' flag.
        LimelightHelpers.PoseEstimate llPose = megaTag2
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(name)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        megaTag2Publisher.set(megaTag2);

        if (llPose != null) {
            // Caller can fuse these with drivetrain odometry using the provided std devs
            estimatedPose = Optional.of(new PoseEstimate(
                    llPose.pose,
                    llPose.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 0.5)
            ));
            posePublisher.set(llPose.pose);
            trackedTargetsPublisher.set(getTargetPoses(llPose.rawFiducials));
        }

        return estimatedPose;
    }

    /**
     * Adds a single condition that, when true, indicates pose updates from this camera
     * should be ignored by consumers.
     *
     * @param condition A boolean supplier representing a disable condition.
     */
    @Override
    public void addDisableCondition(BooleanSupplier condition) {
        disableConditions.add(condition);
    }

    /**
     * Adds multiple conditions that, when any are true, indicate pose updates from this camera
     * should be ignored by consumers.
     *
     * @param conditions A list of boolean suppliers to add to the disable set.
     */
    @Override
    public void addDisableConditions(List<BooleanSupplier> conditions) {
        disableConditions.addAll(conditions);
    }

    /**
     * Checks all registered disable conditions.
     *
     * @return {@code true} if any condition currently requests disabling pose consumption.
     */
    public boolean shouldDisable() {
        for (BooleanSupplier conditon : disableConditions) {
            if (conditon.getAsBoolean()) return true;
        }
        return false;
    }

    /**
     * Enables or disables MegaTag2 mode.
     *
     * @param enable {@code true} to enable MegaTag2; {@code false} to disable.
     */
    public void enableMegaTag2(boolean enable) {
        megaTag2 = enable;
    }

    /**
     * Provides the robot's current yaw to the Limelight to improve pose solving.
     * <p>
     * Internally calls {@link LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double, double)}
     * with all rates set to zero and only yaw supplied.
     *
     * @param rotation Current robot heading as a {@link Rotation2d}.
     */
    public void SetRobotOrientation(Rotation2d rotation) {
        LimelightHelpers.SetRobotOrientation(name, rotation.getDegrees(), 0, 0, 0, 0, 0);
    }

    /**
     * Converts a set of raw fiducials returned by the Limelight into field poses using
     * the robot's loaded {@link FieldConstants#fieldLayout}.
     * <p>
     * If no fiducials are present, returns an empty array.
     *
     * @param tags Array of raw fiducials from the Limelight pose result.
     * @return Array of {@link Pose3d} tag poses from the field layout (same order as input).
     */
    private Pose3d[] getTargetPoses(LimelightHelpers.RawFiducial[] tags) {
        if (tags == null || tags.length == 0) {
            return new Pose3d[0];
        }
        Pose3d[] poses = new Pose3d[tags.length];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tags[i].id).orElse(new Pose3d());
        }
        return poses;
    }
}
