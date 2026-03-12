package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * The {@code GamePieceCamera} class manages a PhotonVision camera that detects
 * 2D objects (such as game pieces) and projects their estimated positions onto
 * the field in 3D space relative to the field coordinate system.
 * <p>
 * This class handles fetching camera data, computing target distances using
 * camera pitch and target pitch, and transforming those detections into
 * {@link Pose3d} locations on the field.
 * </p>
 * <p>
 * The output list of detected targets can be used for visualization, path
 * planning, or mechanism alignment.
 * </p>
 */
public class GamePieceCamera {

    /** PhotonVision camera used for detection. */
    private final PhotonCamera camera;

    /** Supplier providing the current robot pose on the field. */
    private final Supplier<Pose2d> robotPoseSupplier;

    /** Height of the camera lens center above the floor, in meters. */
    private final double cameraHeightMeters;

    /** Pitch of the camera relative to the floor, in radians (positive = up). */
    private final double cameraPitchRad;

    /** Expected target height above the floor, in meters. */
    private final double targetHeightMeters;

    /**
     * Transform representing the position and orientation of the camera relative
     * to the robot frame. (+X forward, +Y left)
     */
    private final Transform2d robotToCamera2d;

    /** Optional cosmetic offset for adjusting the rendered target poses. */
    private final Transform3d visualOffset = new Transform3d();

    /** Cached list of field-relative target poses detected in the latest update. */
    private final ArrayList<Pose3d> detectedGamePieces = new ArrayList<>();
    // Add this new field to hold robot-relative results
    private final ArrayList<Pose3d> robotRelativeGamePieces = new ArrayList<>();

    PhotonPipelineResult resultCashe;

    /**
     * Constructs a new {@code GamePieceCamera}.
     *
     * @param cameraName         The name of the PhotonVision camera (matches
     *                           PhotonVision configuration).
     * @param robotPoseSupplier  A supplier providing the robot's current
     *                           {@link Pose2d} on the field.
     * @param cameraHeightMeters Height of the camera lens center above the floor (m).
     * @param cameraPitchRad     Pitch of the camera relative to the floor (radians).
     *                           Positive values indicate the camera is angled upward.
     * @param targetHeightMeters Expected height of the target above the floor (m).
     * @param robotToCamera2d    The transform from the robot's origin to the
     *                           camera position in the robot coordinate frame.
     */
    public GamePieceCamera(
            String cameraName,
            Supplier<Pose2d> robotPoseSupplier,
            double cameraHeightMeters,
            double cameraPitchRad,
            double targetHeightMeters,
            Transform2d robotToCamera2d
    ) {
        this.camera = new PhotonCamera(cameraName);
        this.robotPoseSupplier = robotPoseSupplier;
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraPitchRad = cameraPitchRad;
        this.targetHeightMeters = targetHeightMeters;
        this.robotToCamera2d = robotToCamera2d;
    }

    /** Shared helper: estimate camera->target translation. Returns null if invalid. */
    private Translation2d estimateCamToTarget(PhotonTrackedTarget target) {
        double targetPitchRad = Units.degreesToRadians(target.getPitch());
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters, targetHeightMeters, cameraPitchRad, targetPitchRad);
        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0) return null;

        // Negate Photon yaw to convert to WPILib convention
        Rotation2d yawFromCamera = Rotation2d.fromDegrees(-target.getYaw());
        return PhotonUtils.estimateCameraToTargetTranslation(distanceMeters, yawFromCamera);
    }

    /**
     * Calculates field-relative 3D positions of all tracked targets in the given
     * list of {@link PhotonTrackedTarget}s.
     * <p>
     * The method uses the current robot pose (from {@code robotPoseSupplier})
     * combined with the known camera transform to derive the camera's field pose.
     * Each target’s pitch and yaw are used to compute a range and heading, which
     * are projected from the camera pose into field coordinates.
     * </p>
     *
     * @param targets The list of tracked targets detected by PhotonVision.
     */
    private void calculateTargetPoses(List<PhotonTrackedTarget> targets) {
        detectedGamePieces.clear();
        robotRelativeGamePieces.clear();
    
        Pose2d fieldRobotPose  = robotPoseSupplier.get();
        Pose2d fieldCameraPose = fieldRobotPose.transformBy(robotToCamera2d);
    
        for (PhotonTrackedTarget target : targets) {
            Translation2d camToTarget = estimateCamToTarget(target);
            if (camToTarget == null) continue;
    
            // FIELD-relative
            Pose2d fieldTarget2d = fieldCameraPose.transformBy(
                    new Transform2d(camToTarget, new Rotation2d()));
            Pose3d fieldTarget3d = new Pose3d(
                    fieldTarget2d.getX(), fieldTarget2d.getY(),
                    targetHeightMeters, new Rotation3d()
            ).plus(visualOffset);
            detectedGamePieces.add(fieldTarget3d);
    
            // ROBOT-relative: (robot->camera) ⊕ (camera->target)
            Transform2d robotToTarget = robotToCamera2d.plus(
                    new Transform2d(camToTarget, new Rotation2d()));
            Pose3d robotTarget3d = new Pose3d(
                    robotToTarget.getX(), robotToTarget.getY(),
                    targetHeightMeters, new Rotation3d()
            ).plus(visualOffset);
            robotRelativeGamePieces.add(robotTarget3d);
        }
    }

    /**
     * Updates the list of detected game pieces using the latest PhotonVision camera result.
     * <p>
     * This method should be called once per robot loop (e.g., inside
     * {@code subsystem.periodic()}). It fetches the latest camera result and, if targets
     * are present, calculates their 3D field poses.
     * </p>
     */
    public void update() {
        var results = camera.getAllUnreadResults();
        if(results.isEmpty()) {
            return;
        }
        PhotonPipelineResult result = results.get(results.size() - 1);
        resultCashe = result;
        if (result.hasTargets()) {
            calculateTargetPoses(result.getTargets());
        } else {
            detectedGamePieces.clear();
            robotRelativeGamePieces.clear();
        }
    }

    /**
     * Returns the current list of field-relative {@link Pose3d}s representing detected game pieces.
     * <p>
     * The list is updated whenever {@link #update()} is called and will contain one entry
     * per target detected by PhotonVision.
     * </p>
     *
     * @return A mutable {@link ArrayList} of {@link Pose3d}s for each detected target.
     */
    public ArrayList<Pose3d> getDetectedGamePieces() {
        return detectedGamePieces;
    }

    /** Robot-relative poses of detected game pieces (Z = target height). */
    public ArrayList<Pose3d> getRobotRelativeGamePieces() {
        return robotRelativeGamePieces;
    }

    public PhotonPipelineResult getResults() {
        return resultCashe;
    }
}
