package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.dashboard.SpikesLogger;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;
import frc.robot.util.LEDService;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class Shoot extends ParallelDeadlineGroup {

    private static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(new Translation2d(0.52, 5.59), new Rotation2d());
    private static final Pose2d RED_SPEAKER_POSE = new Pose2d(new Translation2d(16.07, 5.59), new Rotation2d());

    // formula which translates the distance of the robot from the speaker to the required height to shoot
    private static final Function<Double, Double> DISTANCE_TO_HEIGHT = x -> -3.8 * x + 28.8;
    private static final double MAX_FORMULA_DISTANCE = 2.285;
    private static final double MIN_HEIGHT = 17;

    private static final double STORAGE_VOLTAGE = -6;
    private static final double WAIT_TIME = 0.7;
    private static final double CROSS_TIMEOUT = 1;

    public static final RootNamespace ROOT = new RootNamespace("shoot");
    private static final Supplier<Double> LEFT_SPEED = () -> 3900.0;
    private static final Supplier<Double> RIGHT_SPEED = () -> 3900.0;
    private static final Supplier<Double> HEIGHT = ROOT.addConstantDouble("shoot height", 0);

    public static final double CLOSE_HEIGHT = 24.6;
    public static final double MIDDLE_HEIGHT = 22;
    public static final double RECTANGLE_HEIGHT = 16.3;

    private static final SpikesLogger LOGGER = new SpikesLogger();
    private final double height;

    private final Shooter shooter;
    private final ShooterAdjuster adjuster;
    private final Storage storage;
    private final LEDService ledService;

    private Pose2d speakerPose = new Pose2d(0.52, 5.59, new Rotation2d());

    public Shoot(Shooter shooter, Drivetrain drivetrain, ShooterAdjuster adjuster, Storage storage, double height) {
        super(new InstantCommand());
        this.height = height;
        this.adjuster = adjuster;
        this.shooter = shooter;
        this.storage = storage;
        ledService = LEDService.getInstance();

        SpeedUpShooter speedUpCommand = new SpeedUpShooter(shooter, getRequiredLeftSpeed(), getRequiredRightSpeed());

        MoveSmartMotorControllerGenericSubsystem adjustCommand =
                new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                        adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, getRequiredShooterAngle()) {

                    @Override
                    public boolean isFinished() {
                        return super.isFinished() || !adjuster.wasReset();
                    }
                };

        InstantCommand setSpeakerPoseCommand = new InstantCommand(() -> {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) {
                speakerPose = BLUE_SPEAKER_POSE;
            } else {
                speakerPose = RED_SPEAKER_POSE;
            }
        });

        addCommands(setSpeakerPoseCommand, speedUpCommand, adjustCommand);
        ledService.attemptShoot();
        setDeadline(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                adjustCommand.isFinished() &&
                                        shooter.getLeftFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                shooter.getLeftFlywheel().pidSettings.getTolerance(),
                                                getRequiredLeftSpeed().get()) &&
                                        shooter.getRightFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                shooter.getRightFlywheel().pidSettings.getTolerance(),
                                                getRequiredRightSpeed().get())),
                        new MoveGenericSubsystem(storage, () -> STORAGE_VOLTAGE / RobotController.getBatteryVoltage())
                                .withTimeout(WAIT_TIME).alongWith(new InstantCommand(ledService::shootSuccessful)),
                        new InstantCommand(shooter::stop)
                ));
    }

    public Supplier<Double> getRequiredShooterAngle() {
//        double rx = robotPose.get().getX();
//        double ry = robotPose.get().getY();
//        double sx = speakerPose.getX();
//        double sy = speakerPose.getY();
//        double distance = Math.sqrt((rx - sx) * (rx - sx) + (ry - sy) * (ry - sy));
//        ROOT.putNumber("distance", distance);
//        return distance <= MAX_FORMULA_DISTANCE ? () -> DISTANCE_TO_HEIGHT.apply(distance) : () -> MIN_HEIGHT;
        return () -> height;
    }

    public Supplier<Double> getRequiredLeftSpeed() {
        return LEFT_SPEED;
    }

    public Supplier<Double> getRequiredRightSpeed() {
        return RIGHT_SPEED;
    }

    public Command getCommand() {
        return this.andThen(
                new InstantCommand(shooter::stop, shooter, shooter.getRightFlywheel(), shooter.getLeftFlywheel())
                        .andThen(
                                new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                                        adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                                        IntakeNote.SHOOTER_HEIGHT) {
                                    @Override
                                    public boolean isFinished() {
                                        return super.isFinished() || !adjuster.wasReset();
                                    }
                                }
                        ));
    }
}
