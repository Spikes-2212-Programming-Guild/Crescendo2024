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

/**
 * Command which shoots a note, with parameters controlling the height and speeds of the shooter.
 */
public class ShootWithParameters extends ParallelDeadlineGroup {

    private static final double STORAGE_SPEED = -0.5;
    private static final double WAIT_TIME = 3;

    public static final RootNamespace ROOT = new RootNamespace("shoot with parameters");

    //@TODO CHANGE!
    private static final Pose2d SPEAKER_POSE = new Pose2d(0, 0, new Rotation2d());

    public ShootWithParameters(Shooter shooter, Drivetrain drivetrain, ShooterAdjuster adjuster, Storage storage,
                               Supplier<Double> requiredAngle, Supplier<Double> requiredLeftSpeed,
                               Supplier<Double> requiredRightSpeed, Supplier<Double> requiredHeight) {

        super(new InstantCommand());

//        Pose2d pose = drivetrain.getPose();

        RotateSwerveWithPID rotateCommand = new RotateSwerveWithPID(drivetrain, requiredAngle,
                drivetrain::getAngle, drivetrain.rotateToTargetPIDSettings,
                drivetrain.rotateToTargetFeedForwardSettings) {
            @Override
            public boolean isFinished() {
                return true;
            }
        };

        SpeedUpShooter speedUpCommand = new SpeedUpShooter(shooter, requiredLeftSpeed,
                requiredRightSpeed);

        MoveSmartMotorControllerGenericSubsystem adjustCommand =
                new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                        adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, requiredHeight);

        ROOT.putBoolean("rotate finished", rotateCommand::isFinished);
        ROOT.putBoolean("adjust finished", adjustCommand::isFinished);
        ROOT.putBoolean("left on target", () -> shooter.getLeftFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                shooter.getLeftFlywheel().pidSettings.getTolerance(), requiredLeftSpeed.get()));
        ROOT.putBoolean("right on target", () -> shooter.getRightFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                shooter.getRightFlywheel().pidSettings.getTolerance(), requiredRightSpeed.get()));

        addCommands(speedUpCommand, adjustCommand);
        setDeadline(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                rotateCommand.isFinished() && adjustCommand.isFinished() &&
                                        shooter.getLeftFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                300, requiredLeftSpeed.get()) &&
                                        shooter.getRightFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                300, requiredRightSpeed.get())),
                        new MoveGenericSubsystem(storage, STORAGE_SPEED),
                        new WaitCommand(WAIT_TIME)
                ));
    }
}
