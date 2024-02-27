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

public class PhotonCameraEstimator extends BasePoseSource {

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d cameraToRobotCenter;
    private SpikesLogger logger = new SpikesLogger();

    public PhotonCameraEstimator(String cameraName, Transform3d cameraToRobotCenter) {
        super(cameraToRobotCenter);
        this.photonCamera = new PhotonCamera(cameraName);
        this.cameraToRobotCenter = cameraToRobotCenter;
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
        if (photonCamera.getLatestResult().hasTargets()) {
            return new Pose3d(photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(),
                    photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation());
        }
        return null;
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

    @Override
    protected boolean hasResult() {
        return photonCamera.getLatestResult().hasTargets();
    }

    public int getID() {
        if (hasResult()) return photonCamera.getLatestResult().getBestTarget().getFiducialId();
        return -1;
    }
}
