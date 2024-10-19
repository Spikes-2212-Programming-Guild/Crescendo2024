package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.auto.PathPlannerTest;

import java.util.Set;
import java.util.function.Supplier;

// https://cdn.discordapp.com/attachments/927272978356510721/1167115100117807264/uwuyd99s0cub1.png?ex=654cf3a3&is=653a7ea3&hm=4fd387e2c5dbac2377e7a6c69bceb3218edb077aaeb6685f592d95a89ef7923c&

/**
 * This class represents an MK4 swerve drivetrain.
 */
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
    public static final double RADIUS = FRONT_LEFT_WHEEL_POSITION.getDistance(FRONT_RIGHT_WHEEL_POSITION);

    public static final double MAX_SPEED_METERS_PER_SECONDS = 4.3;
    public static final double MIN_SPEED_METERS_PER_SECONDS = 0.2;

    public final PIDSettings rotateToTargetPIDSettings =
            namespace.addPIDNamespace("rotate to target", new PIDSettings(0.08, 0, 0.0045, 0.6, 0.1));
    public final FeedForwardSettings rotateToTargetFeedForwardSettings =
            namespace.addFeedForwardNamespace("rotate to target", new FeedForwardSettings(0.18, 0));

    private static final String NAMESPACE_NAME = "drivetrain";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Set<SwerveModule> modules;

    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private SpikesLogger logger = new SpikesLogger("drivetrain");

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
        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION, FRONT_RIGHT_WHEEL_POSITION,
                BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions(), new Pose2d());
        modules = Set.of(frontLeft, frontRight, backLeft, backRight);
        configureAutoBuilder();
        configureDashboard();
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(getRotation2d(), getModulePositions());
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed,
                      boolean fieldRelative, boolean usePID, boolean limitSpeed) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                    rotationSpeed, this.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, CENTER_OF_ROBOT);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECONDS);
        frontLeft.set(states[0], usePID, limitSpeed);
        frontRight.set(states[1], usePID, limitSpeed);
        backLeft.set(states[2], usePID, limitSpeed);
        backRight.set(states[3], usePID, limitSpeed);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed,
                      boolean fieldRelative, boolean usePID) {
        drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID, true);
    }

    // overload method created for pathplanner
    public void drive(ChassisSpeeds speeds) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, true, false);
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
        frontLeft.resetRelativeTurnEncoder();
        frontRight.resetRelativeTurnEncoder();
        backLeft.resetRelativeTurnEncoder();
        backRight.resetRelativeTurnEncoder();
    }

    public void resetPosition(Pose2d newPose) {
        odometry.resetPosition(newPose.getRotation(), getModulePositions(), newPose);
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

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(frontRight.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
    }

    public void setAnglesToZero() {
        modules.forEach(module -> module.setAngle(0));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getModulePosition(),
                frontRight.getModulePosition(),
                backLeft.getModulePosition(),
                backRight.getModulePosition()
        };
    }

    private double max(double a, double b, double c, double d) {
        return Math.max(Math.max(a, b), Math.max(c, d));
    }

    private double min(double a, double b, double c, double d) {
        return Math.min(Math.min(a, b), Math.min(c, d));
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
                odometry::getPoseMeters,
                this::resetPosition,
                () -> kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                        backRight.getState()),
                this::drive,
                new HolonomicPathFollowerConfig(
                        MAX_SPEED_METERS_PER_SECONDS,
                        RADIUS,
                        new ReplanningConfig(
                                false, false
                        )
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        logger.log(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? "red" : "blue");
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getAngle);
        namespace.putRunnable("stop drivetrain", this::stop);
        namespace.putNumber("x odom", () -> odometry.getPoseMeters().getX());
        namespace.putNumber("y odom", () -> odometry.getPoseMeters().getY());
        namespace.putNumber("rotation odom", () -> odometry.getPoseMeters().getRotation().getDegrees());
        Supplier<Double> voltage = namespace.addConstantDouble("voltage", 0.2141);
        namespace.putNumber("normalized yaw", this::getNormalizedAngle);
        namespace.putData("set to 0", new RunCommand(this::setAnglesToZero) {
            @Override
            public void end(boolean interrupted) {
                for (SwerveModule module : modules) {
                    module.stop();
                }
            }
        });
        namespace.putNumber("max velocity difference",
                () -> max(Math.abs(frontLeft.getSpeed()), Math.abs(frontRight.getSpeed()),
                        Math.abs(backLeft.getSpeed()),
                        Math.abs(backRight.getSpeed())) - min(Math.abs(frontLeft.getSpeed()),
                        Math.abs(frontRight.getSpeed()), Math.abs(backLeft.getSpeed()),
                        Math.abs(backRight.getSpeed())));
        Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
        namespace.putData("drive in speed i tell you", new DriveSwerve(this, setpoint, () -> 0.0, () -> 0.0, false,
                true));
        namespace.putNumber("rotation speed", gyro::getRawGyroZ);
        namespace.putData("test pathplanner", new InstantCommand(() -> odometry.resetPosition(gyro.getRotation2d(),
                getModulePositions(), new Pose2d(0, 2, new Rotation2d()))).andThen(
                new PathPlannerTest(PathPlannerPath.fromPathFile("lol"), this)));
    }
}