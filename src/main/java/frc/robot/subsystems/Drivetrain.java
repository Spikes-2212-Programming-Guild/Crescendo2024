package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.Limelight;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

// https://cdn.discordapp.com/attachments/927272978356510721/1167115100117807264/uwuyd99s0cub1.png?ex=654cf3a3&is=653a7ea3&hm=4fd387e2c5dbac2377e7a6c69bceb3218edb077aaeb6685f592d95a89ef7923c&
public class Drivetrain extends DashboardedSubsystem {

    public static final double TRACK_WIDTH = 0.59;

    public static final Translation2d FRONT_LEFT_WHEEL_POSITION =
            new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_WHEEL_POSITION =
            new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BACK_LEFT_WHEEL_POSITION =
            new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_WHEEL_POSITION =
            new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2);
    public static final Translation2d CENTER_OF_ROBOT = new Translation2d(
            (FRONT_LEFT_WHEEL_POSITION.getX() + BACK_RIGHT_WHEEL_POSITION.getX()) / 2,
            (FRONT_LEFT_WHEEL_POSITION.getY() + BACK_RIGHT_WHEEL_POSITION.getY()) / 2);

    public static final double MAX_SPEED_METERS_PER_SECONDS = 4.7;
    public static final double MIN_SPEED_METERS_PER_SECONDS = 0.2;

    private static final String NAMESPACE_NAME = "drivetrain";

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule backLeft;
    public final SwerveModule backRight;

    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Limelight limelight;
    private final PhotonCamera photonCamera;

    private final PhotonPoseEstimator poseEstimator;
    private Pose2d robotPose = new Pose2d();

    private SwerveModulePosition[] modulePositions;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(SwerveModuleHolder.getFrontLeft(), SwerveModuleHolder.getFrontRight(),
                    SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), new AHRS(SerialPort.Port.kMXP));
        }
        return instance;
    }

    private Drivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                       SwerveModule backRight, AHRS gyro) {
        super(NAMESPACE_NAME);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION, FRONT_RIGHT_WHEEL_POSITION,
                BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
        this.gyro = gyro;
        limelight = new Limelight("limelight-back");
        photonCamera = new PhotonCamera("photonvision");
        modulePositions = new SwerveModulePosition[]
                {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(),
                        backRight.getModulePosition()};
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), modulePositions, new Pose2d());
        PhotonPoseEstimator poseEstimator1;
        try {
            poseEstimator1 = new PhotonPoseEstimator(
                    AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE, photonCamera,
                    new Transform3d(new Translation3d(0, 0.36, 0), new Rotation3d()));
        } catch (Exception e) {
            poseEstimator1 = null;
        }
        poseEstimator = poseEstimator1;
        configureDashboard();
    }

    @Override
    public void periodic() {
        super.periodic();
        modulePositions = new SwerveModulePosition[]
                {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(),
                        backRight.getModulePosition()};
        odometry.update(getRotation2d(), modulePositions);
        poseEstimator.setReferencePose(odometry.getPoseMeters());
        if (photonCamera.getLatestResult().hasTargets()) {
            robotPose = odometry.getPoseMeters();
            poseEstimator.setLastPose(robotPose);
        } else {
            try {
                robotPose = poseEstimator.update().get().estimatedPose.toPose2d();
                poseEstimator.setLastPose(robotPose);
            } catch (Exception ignored) {
            }
        }
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed,
                      boolean fieldRelative, boolean usePID) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                    rotationSpeed, this.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, CENTER_OF_ROBOT);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECONDS);
        frontLeft.set(states[0], usePID);
        frontRight.set(states[1], usePID);
        backLeft.set(states[2], usePID);
        backRight.set(states[3], usePID);
    }

    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Resets the position to the position obtained from the vision source, or the given position if there is no
     * {@link AprilTag} target.
     *
     * @param noTargetPosition the position to reset the estimator to if there is no information from the vision source.
     * @TODO add to {@link frc.robot.Robot}
     */
    public void resetPoseEstimator(Pose2d noTargetPosition) {
//        poseEstimator.resetPosition(
//                gyro.getRotation2d(),
//                new SwerveModulePosition[]{
//                        frontLeft.getPosition(),
//                        frontRight.getPosition(),
//                        backLeft.getPosition(),
//                        backRight.getPosition()
//                },
//                limelight.hasTarget() ? limelight.getRobotPose().toPose2d() : noTargetPosition
//        );

    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void pain() {
        frontLeft.set(new SwerveModuleState(0.7, new Rotation2d(0)), false);
        frontRight.set(new SwerveModuleState(0.7, new Rotation2d(0)), false);
        backLeft.set(new SwerveModuleState(0.7, new Rotation2d(0)), false);
        backRight.set(new SwerveModuleState(0.7, new Rotation2d(0)), false);
    }

    public void resetRelativeEncoders() {
        frontLeft.configureRelativeTurnEncoder();
        frontRight.configureRelativeTurnEncoder();
        backLeft.configureRelativeTurnEncoder();
        backRight.configureRelativeTurnEncoder();
    }

    public double getAngle() {
        return -gyro.getAngle();
    }

    public double getNormalizedAngle() {
        return getRotation2d().getDegrees() % 180;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public void iAmDeathDestroyerOfWorlds() {
        double velocity = 0.015;
        frontRight.setAngle(45);
        frontLeft.setAngle(135);
        backLeft.setAngle(225);
        backRight.setAngle(315);
        frontLeft.driveController.set(velocity);
        frontRight.driveController.set(velocity);
        backLeft.driveController.set(velocity);
        backRight.driveController.set(velocity);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getAngle);
        namespace.putRunnable("stop drivetrain", () -> stop());
        namespace.putRunnable("move PLEWA,SP;RKW.OIA", this::pain);
        namespace.putNumber("x pos", () -> robotPose.getX());
        namespace.putNumber("y pos", () -> robotPose.getY());
        namespace.putNumber("angle", () -> robotPose.getRotation().getDegrees());
        namespace.putNumber("x odom", () -> odometry.getPoseMeters().getX());
        namespace.putNumber("y odom", () -> odometry.getPoseMeters().getY());
        namespace.putNumber("rotation odom", () -> odometry.getPoseMeters().getRotation().getDegrees());
        namespace.putNumber("normalized yaw",  this::getNormalizedAngle);
    }
}
