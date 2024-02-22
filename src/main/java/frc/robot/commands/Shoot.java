package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;


public class Shoot extends ParallelDeadlineGroup {


    private static final double STORAGE_SPEED = -0.5;
    private static final double WAIT_TIME = 3;

    public static final RootNamespace ROOT = new RootNamespace("shoot");
    private static final Supplier<Double> LEFT_SPEED = ROOT.addConstantDouble("shoot left speed", 0);
    private static final Supplier<Double> RIGHT_SPEED = ROOT.addConstantDouble("shoot right speed", 0);
    private static final Supplier<Double> HEIGHT = ROOT.addConstantDouble("shoot height", 0);

    //@TODO CHANGE!
    private static final Pose2d SPEAKER_POSE = new Pose2d(0, 0, new Rotation2d());

    public Shoot(Shooter shooter, Drivetrain drivetrain, ShooterAdjuster adjuster, Storage storage) {

        super(new InstantCommand());

        Pose2d pose = drivetrain.getPose();

        RotateSwerveWithPID rotateCommand = new RotateSwerveWithPID(drivetrain, () -> getRequiredShooterAngle(pose),
                drivetrain::getAngle, drivetrain.rotateToTargetPIDSettings,
                drivetrain.rotateToTargetFeedForwardSettings) {
            @Override
            public boolean isFinished() {
                return true;
            }
        };

        SpeedUpShooter speedUpCommand = new SpeedUpShooter(shooter, () -> getRequiredLeftSpeed(pose),
                () -> getRequiredRightSpeed(pose));

        MoveSmartMotorControllerGenericSubsystem adjustCommand =
                new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                        adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                        () -> getRequiredShooterAngle(pose)) {
                };

        ROOT.putBoolean("rotate finished", rotateCommand::isFinished);
        ROOT.putBoolean("adjust finished", adjustCommand::isFinished);
        ROOT.putBoolean("left on target", () -> shooter.getLeftFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                shooter.getLeftFlywheel().pidSettings.getTolerance(), getRequiredLeftSpeed(pose)));
        ROOT.putBoolean("right on target", () -> shooter.getRightFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                shooter.getRightFlywheel().pidSettings.getTolerance(), getRequiredRightSpeed(pose)));

        addCommands(speedUpCommand, adjustCommand);
        setDeadline(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                rotateCommand.isFinished() && adjustCommand.isFinished() &&
                                        shooter.getLeftFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                shooter.getLeftFlywheel().pidSettings.getTolerance(), getRequiredLeftSpeed(pose)) &&
                                        shooter.getRightFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                shooter.getRightFlywheel().pidSettings.getTolerance(), getRequiredRightSpeed(pose))),
                        new MoveGenericSubsystem(storage, STORAGE_SPEED),
                        new WaitCommand(WAIT_TIME)
                ));
    }

    public double getRequiredShooterAngle(Pose2d pose) {
//        return HEIGHT.get();
        return 24.8;
    }

    public double getRequiredRobotAngle(Pose2d robotPose) {
        return Math.atan2(SPEAKER_POSE.minus(robotPose).getX(), SPEAKER_POSE.minus(robotPose).getY());
    }

    public double getRequiredLeftSpeed(Pose2d pose) {
//        return LEFT_SPEED.get();
        return 4000;
    }

    public double getRequiredRightSpeed(Pose2d pose) {
//        return RIGHT_SPEED.get();
        return 4000;
    }
}
