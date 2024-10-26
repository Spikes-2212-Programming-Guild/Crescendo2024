package frc.robot.services.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

//https://www.google.com/url?sa=i&url=https%3A%2F%2Fbr.ifunny.co%2Fpicture%2Fthank-you-for-50-followers-EBR5wbN09&psig=AOvVaw2RYeu6jPFY5xituY0gxZ6Z&ust=1706021378549000&source=images&cd=vfe&opi=89978449&ved=0CBIQjRxqFwoTCJih2YSf8YMDFQAAAAAdAAAAABAD

/**
 * Class which represents a camera which is used to get the robot pose.
 */
public abstract class BasePoseSource {

    protected final Transform3d cameraToRobotCenter;
    private double lastUpdatedTimestamp;
    private Pose2d lastPose;

    protected BasePoseSource(Transform3d cameraToRobotCenter) {
        this.cameraToRobotCenter = cameraToRobotCenter;
    }

    public Pose2d getRobotPose() {
        Pose3d cameraPose = getCameraPose();
        if (cameraPose == null) return lastPose;
        lastPose = new Pose3d(cameraPose.getTranslation().plus(cameraToRobotCenter.getTranslation()),
                cameraPose.getRotation().plus(cameraToRobotCenter.getRotation())).toPose2d();
        return lastPose;
    }

    public abstract double getLastResultTimestamp();

    public abstract String getName();

    public boolean hasNewResult() {
        return isNewTimestamp() && hasResult();
    }

    protected abstract Pose3d getCameraPose();

    protected abstract boolean hasResult();

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp()) return false;
        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }
}
