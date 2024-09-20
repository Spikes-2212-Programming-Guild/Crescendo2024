package frc.robot.services.poseestimation;

import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;

/**
 * Class which represents the cameras placed on the robot.
 */
public class PoseEstimationCameras {

    private static final String CENTER_CAMERA_NAME = "center";
    private static final String LEFT_CAMERA_NAME = "left";
    private static final String RIGHT_CAMERA_NAME = "right";

    private static final Transform3d CENTER_CAMERA_LOCATION = new Transform3d(new Translation3d(-0.205, -0.27, 0.685),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-40), Math.toRadians(0)));
    //checked translation
    private static final Transform3d LEFT_CAMERA_LOCATION = new Transform3d(new Translation3d(-0.145, -0.325, 0.685),
            new Rotation3d(Math.toRadians(20), Math.toRadians(0), Math.toRadians(90)).unaryMinus());
    private static final Transform3d RIGHT_CAMERA_LOCATION = new Transform3d();

    private final List<PhotonCameraEstimator> estimators;
    private final SpikesLogger logger = new SpikesLogger();

    private List<PoseEstimatorTarget> results;

    private static PoseEstimationCameras instance;

    public static PoseEstimationCameras getInstance() {
        if (instance == null) {
            instance = new PoseEstimationCameras();
        }
        return instance;
    }

    private PoseEstimationCameras() {
//        PhotonCameraEstimator centerEstimator = new PhotonCameraEstimator(CENTER_CAMERA_NAME, CENTER_CAMERA_LOCATION);
//        PhotonCameraEstimator leftEstimator = new PhotonCameraEstimator(LEFT_CAMERA_NAME, LEFT_CAMERA_LOCATION);
//        PhotonCameraEstimator rightEstimator = new PhotonCameraEstimator(RIGHT_CAMERA_NAME, RIGHT_CAMERA_LOCATION);
        //@TODO CHANGE
        estimators = List.of();
        results = new ArrayList<>();
    }

    public List<PoseEstimatorTarget> getEstimatedPoses() {
        results = new ArrayList<>();
        for (PhotonCameraEstimator photonCameraEstimator : estimators) {
            if (photonCameraEstimator != null) {
                if (photonCameraEstimator.hasResult()) {
                    results.add(new PoseEstimatorTarget(photonCameraEstimator.getRobotPose(),
                            photonCameraEstimator.getLastResultTimestamp()));
                }
            }
        }
        return results;
    }
}
