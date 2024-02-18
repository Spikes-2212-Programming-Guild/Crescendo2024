package frc.robot.services.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.List;

public class PoseEstimationCameras {

    private static final String PHOTON_CAMERA_1_NAME = "jimmy";
    private static final String PHOTON_CAMERA_2_NAME = "jamie";
    private static final String PHOTON_CAMERA_3_NAME = "bob";

    private static final Transform3d CAMERA_1_LOCATION = new Transform3d();
    private static final Transform3d CAMERA_2_LOCATION = new Transform3d();
    private static final Transform3d CAMERA_3_LOCATION = new Transform3d();

    private final PhotonCameraEstimator photonPoseEstimator1;
    private final PhotonCameraEstimator photonPoseEstimator2;
    private final PhotonCameraEstimator photonPoseEstimator3;

    private final List<PhotonCameraEstimator> estimators;
    private List<Pose2d> results;

    private PoseEstimationCameras() {
        this.photonPoseEstimator1 = new PhotonCameraEstimator(PHOTON_CAMERA_1_NAME, CAMERA_1_LOCATION);
        this.photonPoseEstimator2 = new PhotonCameraEstimator(PHOTON_CAMERA_2_NAME, CAMERA_2_LOCATION);
        this.photonPoseEstimator3 = new PhotonCameraEstimator(PHOTON_CAMERA_3_NAME, CAMERA_3_LOCATION);
        estimators = List.of(photonPoseEstimator1, photonPoseEstimator2, photonPoseEstimator3);
        results = new ArrayList<>();
    }

    public List<Pose2d> getEstimatedPoses() {
        results = new ArrayList<>();
        for (PhotonCameraEstimator photonCameraEstimator : estimators) {
            if (photonCameraEstimator != null) {
                if (photonCameraEstimator.hasResult()) {
                    results.add(photonCameraEstimator.getRobotPose());
                }
            }
        }
        return results;
    }
}
