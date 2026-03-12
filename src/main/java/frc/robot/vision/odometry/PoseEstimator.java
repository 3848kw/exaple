package frc.robot.vision.odometry;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import swervelib.SwerveDrive;

public class PoseEstimator {
    private VisionSystemSim visionSim;
    private ArrayList<OdometryCamera> cameras;
    private Supplier<Pose2d> getSimDrivetrainPose;

    /**
     * Creates a new Pose Estimator
     * @param getSimPose The Pose of the robot in simulation only. This should be seperate from the physical pose
     */
    public PoseEstimator(Supplier<Pose2d> getSimPose) {
        visionSim = new VisionSystemSim("Vision Sim");
        visionSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());
        this.cameras = new ArrayList<OdometryCamera>();
        this.getSimDrivetrainPose = getSimPose;
    }

    /**
     * Adds a camera to the Pose Estimator
     * @param camera
     */
    public void addCamera(OdometryCamera camera) {
        cameras.add(camera);

        if(camera instanceof PhotonOdometryCamera) {
            PhotonOdometryCamera cam = (PhotonOdometryCamera) camera;
            visionSim.addCamera(cam.getSimCamera(), cam.getRobotToCam());
        }
    }

    /**
     * Adds a list of cameras to the Pose Estimator
     * @param cameras
     */
    public void addCameras(OdometryCamera... cameras) {
        for(OdometryCamera camera : cameras) {
            addCamera(camera);
        }
    }

    /**
     * Update the robots pose from vision
     * @param swerveDrive the YAGSL SwerveDrive object
     */
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        //update pose estimations from photon cameras
        if(Robot.isSimulation()) visionSim.update(getSimDrivetrainPose.get());
        for(OdometryCamera camera : cameras) {
            if(camera instanceof LimelightOdometryCamera) {
                ((LimelightOdometryCamera)camera).SetRobotOrientation(swerveDrive.getOdometryHeading());
            }
            Optional<PoseEstimate> poseEstimate = camera.update();
            if(poseEstimate.isPresent()) {
                swerveDrive.addVisionMeasurement(poseEstimate.get().poseEstimate, poseEstimate.get().timeStamp, poseEstimate.get().stdDevs);
            }
        }
    }
}
