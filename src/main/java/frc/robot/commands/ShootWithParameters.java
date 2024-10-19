package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.*;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
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
    private static final double LEFT_TOLERANCE = 300;
    private static final double RIGHT_TOLERANCE = 300;

    public static final RootNamespace ROOT = new RootNamespace("shoot with parameters");

    public ShootWithParameters(Shooter shooter, Drivetrain drivetrain, ShooterAdjuster adjuster, Storage storage,
                               Supplier<Double> requiredAngle, Supplier<Double> requiredLeftSpeed,
                               Supplier<Double> requiredRightSpeed, Supplier<Double> requiredHeight) {
        super(new InstantCommand());

        RotateSwerveWithPID rotateCommand = new RotateSwerveWithPID(drivetrain, requiredAngle,
                drivetrain::getAngle, drivetrain.rotateToTargetPIDSettings,
                drivetrain.rotateToTargetFeedForwardSettings);

        SpeedUpShooter speedUpCommand = new SpeedUpShooter(shooter, requiredLeftSpeed,
                requiredRightSpeed);

        MoveSmartMotorControllerGenericSubsystem adjustCommand =
                new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                        adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, requiredHeight);

        addCommands(speedUpCommand, adjustCommand);
        setDeadline(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> rotateCommand.isFinished() && adjustCommand.isFinished() &&
                                        shooter.getLeftFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                LEFT_TOLERANCE, requiredLeftSpeed.get()) &&
                                        shooter.getRightFlywheel().onTarget(UnifiedControlMode.VELOCITY,
                                                RIGHT_TOLERANCE, requiredRightSpeed.get())),
                        new MoveGenericSubsystem(storage, STORAGE_SPEED),
                        new WaitCommand(WAIT_TIME)
                ));
    }
}
