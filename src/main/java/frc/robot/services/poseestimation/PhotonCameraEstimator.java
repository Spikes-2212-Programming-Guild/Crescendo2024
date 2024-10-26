package frc.robot.services.poseestimation;

import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import java.io.IOException;

/**
 * Class which represents a camera running photonvision which is used to get the robot pose.
 */
public class PhotonCameraEstimator extends BasePoseSource {

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private SpikesLogger logger = new SpikesLogger();

    public PhotonCameraEstimator(Transform3d cameraToRobotCenter, String cameraName) {
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
    public Pose2d getRobotPose() {
        if (getCameraPose() != null) {
            if (poseEstimator.getFieldTags().getTagPose(getID()).isPresent()) {
                return PhotonUtils.estimateFieldToRobotAprilTag(new Transform3d(getCameraPose().getTranslation(),
                                getCameraPose().getRotation()),
                        poseEstimator.getFieldTags().getTagPose(getID()).get(), cameraToRobotCenter).toPose2d();
            }
        }
        return null;
    }

    public int getID() {
        if (hasResult()) return photonCamera.getLatestResult().getBestTarget().getFiducialId();
        return -1;
    }

    @Override
    protected boolean hasResult() {
        return photonCamera.getLatestResult().hasTargets();
    }

    @Override
    protected Pose3d getCameraPose() {
        if (photonCamera.getLatestResult().hasTargets()) {
            return new Pose3d(photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(),
                    photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation());
        }
        return null;
    }
}
