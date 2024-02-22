package frc.robot.services.poseestimation;

import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

import java.util.ArrayList;
import java.util.List;

public class PoseEstimationCameras {

    private static final String PHOTON_CAMERA_1_NAME = "jimmy";
    private static final String PHOTON_CAMERA_2_NAME = "jamie";
//    private static final String PHOTON_CAMERA_3_NAME = "bob";

    private static final Transform3d CAMERA_1_LOCATION = new Transform3d(new Translation3d(-27.625, 36.25, 64.3),
            new Rotation3d(Math.toRadians(0), Math.toRadians(50), Math.toRadians(70)));
    private static final Transform3d CAMERA_2_LOCATION = new Transform3d(new Translation3d(-23.5, 32.25, 62),
            new Rotation3d(Math.toRadians(85), Math.toRadians(7), Math.toRadians(-10)));
    private static final Transform3d CAMERA_3_LOCATION = new Transform3d();

    private final List<PhotonCameraEstimator> estimators;
    private List<PoseEstimatorTarget> results;

    private SpikesLogger logger = new SpikesLogger();

    private static PoseEstimationCameras instance;

    public static PoseEstimationCameras getInstance() {
        if (instance == null) {
            instance = new PoseEstimationCameras();
        }
        return instance;
    }

    private PoseEstimationCameras() {
        PhotonCameraEstimator photonPoseEstimator1 = new PhotonCameraEstimator(PHOTON_CAMERA_1_NAME, CAMERA_1_LOCATION);
        PhotonCameraEstimator photonPoseEstimator2 = new PhotonCameraEstimator(PHOTON_CAMERA_2_NAME, CAMERA_2_LOCATION);
//        PhotonCameraEstimator photonPoseEstimator3 = new PhotonCameraEstimator(PHOTON_CAMERA_3_NAME, CAMERA_3_LOCATION);
        estimators = List.of(photonPoseEstimator1, photonPoseEstimator2);
        results = new ArrayList<>();
    }

    public List<PoseEstimatorTarget> getEstimatedPoses() {
        results = new ArrayList<>();
        for (PhotonCameraEstimator photonCameraEstimator : estimators) {
            if (photonCameraEstimator != null) {
                if (photonCameraEstimator.hasResult()) {
                    logger.log("Robot: " + photonCameraEstimator.getRobotPose());
                    results.add(new PoseEstimatorTarget(photonCameraEstimator.getRobotPose(),
                            photonCameraEstimator.getLastResultTimestamp()));
                }
            }
        }
        return results;
    }
}
