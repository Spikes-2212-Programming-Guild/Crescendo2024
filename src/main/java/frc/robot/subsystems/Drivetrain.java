package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateSwerveWithPID;
import frc.robot.services.poseestimation.PoseEstimationCameras;
import frc.robot.services.poseestimation.PoseEstimatorTarget;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

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

    public static final double MAX_SPEED_METERS_PER_SECONDS = 4.3;
    public static final double MIN_SPEED_METERS_PER_SECONDS = 0.2;

    private static final String NAMESPACE_NAME = "drivetrain";

    public final PIDSettings rotateToTargetPIDSettings =
            namespace.addPIDNamespace("rotate to target", new PIDSettings(0.08, 0, 0.0045, 0.6, 0.1));
    public final FeedForwardSettings rotateToTargetFeedForwardSettings =
            namespace.addFeedForwardNamespace("rotate to target", new FeedForwardSettings(0.18, 0));

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Set<SwerveModule> modules;

    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final PoseEstimationCameras poseEstimationCameras;
    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveModulePosition[] modulePositions;

    private double maxFrontLeftSpeed = 0;
    private double maxBackLeftSpeed = 0;
    private double maxFrontRightSpeed = 0;
    private double maxBackRightSpeed = 0;
    private double maxRotationSpeed = 0;
    private double maxAcceleration = 0;

    private SpikesLogger logger = new SpikesLogger();

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
        modules = Set.of(frontLeft, frontRight, backLeft, backRight);
        poseEstimationCameras = PoseEstimationCameras.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), modulePositions, new Pose2d());
        configureDashboard();
    }

    @Override
    public void periodic() {
        super.periodic();
        modulePositions = new SwerveModulePosition[]
                {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(),
                        backRight.getModulePosition()};
        odometry.update(getRotation2d(), modulePositions);
        poseEstimator.update(getRotation2d(), modulePositions);
        List<PoseEstimatorTarget> targets = poseEstimationCameras.getEstimatedPoses();
//        for (PoseEstimatorTarget target : targets) {
//            if (target != null) {
//                if (target.getPose() != null) {
//                    poseEstimator.addVisionMeasurement(target.getPose(), target.getTimestamp());
//                    new SpikesLogger().log(target.getPose());
//                }
//                break;
//            }
//        }
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

//    /**
//     * Minimize the change in heading the desired swerve module state would require by potentially
//     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
//     * in appropriate scope for REV onboard control.
//     * Credit to team #364.
//     *
//     * @param desiredState the desired state
//     * @param currentAngle the current module angle
//     */
//    private double optimize(Rotation2d desiredAngle, Rotation2d currentAngle) {
//        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredAngle.getDegrees());
//        double targetSpeed = desiredState.speedMetersPerSecond;
//        double delta = targetAngle - currentAngle.getDegrees();
//        if (Math.abs(delta) > 180) {
//            targetSpeed = -targetSpeed;
//            targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
//        }
//        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
//    }

    /**
     * Takes the module's angle and the desired angle, and returns it in within the scope reference and 360 degrees
     * above it.
     * Credit to team #364.
     *
     * @param scopeReference current angle
     * @param newAngle       target angle
     * @return closest angle within scope
     */
    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public double getNormalizedAngle() {
        return getRotation2d().getDegrees() % 180;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    private double max(double a, double b, double c, double d) {
        return Math.max(Math.max(a, b), Math.max(c, d));
    }

    private double min(double a, double b, double c, double d) {
        return Math.min(Math.min(a, b), Math.min(c, d));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getAngle);
        namespace.putRunnable("stop drivetrain", this::stop);
        namespace.putNumber("x odom", () -> odometry.getPoseMeters().getX());
        namespace.putNumber("y odom", () -> odometry.getPoseMeters().getY());
        namespace.putNumber("rotation odom", () -> odometry.getPoseMeters().getRotation().getDegrees());
//        namespace.putNumber("x est", () -> getPose().getX());
//        namespace.putNumber("y est", () -> getPose().getY());
//        namespace.putNumber("rotation est", () -> getPose().getRotation().getDegrees());
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
                () -> max(Math.abs(frontLeft.getSpeed()), Math.abs(frontRight.getSpeed()), Math.abs(backLeft.getSpeed()),
                        Math.abs(backRight.getSpeed())) - min(Math.abs(frontLeft.getSpeed()), Math.abs(frontRight.getSpeed()), Math.abs(backLeft.getSpeed()),
                        Math.abs(backRight.getSpeed())));
        Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
        namespace.putCommand("rotate to angle", new RotateSwerveWithPID(this,
                () -> placeInAppropriate0To360Scope(getAngle(), setpoint.get()), this::getAngle, rotateToTargetPIDSettings,
                rotateToTargetFeedForwardSettings));
        namespace.putData("find max drive velocity (good idea)", new RunCommand(this::driveFast) {
            @Override
            public void end(boolean interrupted) {
                stop();
            }
        });
        namespace.putData("find max rotation velocity (probalbly safer)",
                new SequentialCommandGroup(
                        new FunctionalCommand(() -> {
                        }, this::diamond, b -> stop(), () -> false, this).withTimeout(0.5),
                        new FunctionalCommand(() -> {
                        }, this::driveFast,
                                b -> stop(), () -> false, this)));
        namespace.putNumber("max fl speed", () -> maxFrontLeftSpeed);
        namespace.putNumber("max fr speed", () -> maxFrontRightSpeed);
        namespace.putNumber("max bl speed", () -> maxBackLeftSpeed);
        namespace.putNumber("max br speed", () -> maxBackRightSpeed);
        namespace.putNumber("max accel", () -> maxAcceleration);
        namespace.putNumber("max rotation speed", () -> maxRotationSpeed);
        namespace.putData("set voltage", new RunCommand(() -> setVoltage(2.5), this, frontLeft, frontRight, backLeft, backRight));
//        namespace.putData("test pathplanner", new PathPlannerTest(PathPlannerPath.fromPathFile("New Path"), this));
//        namespace.putRunnable("set pose", () -> poseEstimator.resetPosition(gyro.getRotation2d(), modulePositions,
//                poseEstimationCameras.getEstimatedPoses().get(0).getPose()));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
//        return null;
    }


    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(frontRight.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
    }

    public void driveFast() {
        for (SwerveModule module : modules) {
            module.driveController.set(1);
        }
        maxFrontLeftSpeed = Math.max(maxFrontLeftSpeed, Math.abs(frontLeft.getSpeed()));
        maxFrontRightSpeed = Math.max(maxFrontRightSpeed, Math.abs(frontRight.getSpeed()));
        maxBackLeftSpeed = Math.max(maxBackLeftSpeed, Math.abs(backLeft.getSpeed()));
        maxBackRightSpeed = Math.max(maxBackRightSpeed, Math.abs(backRight.getSpeed()));
        maxRotationSpeed = Math.max(maxRotationSpeed, Math.abs(gyro.getRate()));
        //maybe
        maxAcceleration = Math.max(maxAcceleration, Math.abs(gyro.getWorldLinearAccelX()));
    }

    public void beyblade() {
        for (SwerveModule module : modules) {
            module.driveController.set(1);
        }
    }

    public void setAnglesToZero() {
        modules.forEach(m -> m.setAngle(0));
    }

    public void diamond() {
        frontLeft.setAngle(135);
        frontRight.setAngle(45);
        backLeft.setAngle(45);
        backRight.setAngle(135);
    }

    public void crossMode() {
        frontLeft.setAngle(45);
        frontRight.setAngle(135);
        backLeft.setAngle(135);
        backRight.setAngle(45);
    }

    public void setVoltage(double voltage) {
        modules.forEach(m -> m.driveController.setVoltage(voltage));
    }
}