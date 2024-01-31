package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

public class Shoot extends ParallelDeadlineGroup {

    private static final double STORAGE_SPEED = 0;
    private static final double WAIT_TIME = 0;

    //@TODO CHANGE!
    private static final Pose2d SPEAKER_POSE  = new Pose2d(0, 0, new Rotation2d());

    public Shoot(Shooter shooter, Drivetrain drivetrain, ShooterAdjuster adjuster, Storage storage) {
        super(new InstantCommand());

        Pose2d pose = drivetrain.getPose();

        RotateSwerveWithPID rotateCommand = new RotateSwerveWithPID(drivetrain, () -> getRequiredRobotAngle(pose),
                drivetrain::getAngle, drivetrain.getPIDSettings(), drivetrain.getFeedForwardSettings());

        SpeedUpShooter speedUpCommand = new SpeedUpShooter(shooter, () -> getRequiredLeftSpeed(pose),
                () -> getRequiredRightSpeed(pose));

        MoveSmartMotorControllerSubsystemTrapezically adjustCommand =
                new MoveSmartMotorControllerSubsystemTrapezically(adjuster, adjuster.getPIDSettings(),
                        adjuster.getFeedForwardSettings(), () -> getRequiredShooterAngle(pose),
                        adjuster.getTrapezoidProfileSettings());

        addCommands(rotateCommand, speedUpCommand, adjustCommand);
        setDeadline(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                rotateCommand.isFinished() && adjustCommand.isFinished() &&
                                        shooter.getLeftFlywheel().onTarget(UnifiedControlMode.TRAPEZOID_PROFILE,
                                                shooter.leftPIDSettings.getTolerance(), getRequiredLeftSpeed(pose)) &&
                                        shooter.getRightFlywheel().onTarget(UnifiedControlMode.TRAPEZOID_PROFILE,
                                                shooter.rightPIDSettings.getTolerance(), getRequiredRightSpeed(pose))),
                        new MoveGenericSubsystem(storage, STORAGE_SPEED),
                        new WaitCommand(WAIT_TIME)
                ));
    }

    public double getRequiredShooterAngle(Pose2d pose) {
        return 0.0;
    }

    public double getRequiredRobotAngle(Pose2d robotPose) {
        return Math.atan2(SPEAKER_POSE.minus(robotPose).getX(), SPEAKER_POSE.minus(robotPose).getY());
    }

    public double getRequiredLeftSpeed(Pose2d pose) {
        return 0.0;
    }

    public double getRequiredRightSpeed(Pose2d pose) {
        return 0.0;
    }
}