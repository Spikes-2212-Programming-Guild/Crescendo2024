package frc.robot.services.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseEstimatorTarget {

    private final Pose2d pose;
    private final double timestamp;

    public PoseEstimatorTarget(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTimestamp() {
        return timestamp;
    }
}
