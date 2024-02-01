package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort;

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

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

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
        modulePositions = new SwerveModulePosition[]
                {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(),
                        backRight.getModulePosition()};
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), modulePositions, new Pose2d());
        configureDashboard();
    public PIDSettings getPIDSettings(){
        return null;
    }
    public FeedForwardSettings getFeedForwardSettings(){
        return null;
    }

    public Pose2d getPose(){
        return null;
    }
    public double getAngle() {
        return 0;
    }
    // using mcs
    @Override
    public void periodic() {
        super.periodic();
        modulePositions = new SwerveModulePosition[]
                {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(),
                        backRight.getModulePosition()};
        odometry.update(getRotation2d(), modulePositions);
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

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
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

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getAngle);
        namespace.putRunnable("stop drivetrain", () -> stop());
        namespace.putNumber("x odom", () -> odometry.getPoseMeters().getX());
        namespace.putNumber("y odom", () -> odometry.getPoseMeters().getY());
        namespace.putNumber("rotation odom", () -> odometry.getPoseMeters().getRotation().getDegrees());
        namespace.putNumber("normalized yaw", this::getNormalizedAngle);
    }
}
