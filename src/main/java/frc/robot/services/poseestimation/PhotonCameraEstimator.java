package frc.robot.services.poseestimation;

import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.Optional;

public class PhotonCameraEstimator extends BasePoseSource {

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonCameraEstimator(String cameraName, Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
        this.photonCamera = new PhotonCamera(cameraName);
        PhotonPoseEstimator temp;
        try {
            temp = new PhotonPoseEstimator(
                    AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE, photonCamera, cameraToRobotCenter);
        } catch (IOException e) {
            temp = null;
        }
        poseEstimator = temp;
    }

    @Override
    public double getLastResultTimestamp() {
        return photonCamera.getLatestResult().getTimestampSeconds();
    }

    @Override
    public String getName() {
        return photonCamera.getName();
    }

    @Override
    protected Pose3d getCameraPose() {
        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
        return estimatedRobotPose.map(robotPose -> robotPose.estimatedPose).orElse(null);
    }

    @Override
    protected boolean hasResult() {
        return photonCamera.getLatestResult().hasTargets();
    }
}
