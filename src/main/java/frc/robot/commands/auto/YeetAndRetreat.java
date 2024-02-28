package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class YeetAndRetreat extends SequentialCommandGroup {

    private static final double MOVE_TIMEOUT = 4;
    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;

    //Click A-Stop if necessary!
    public YeetAndRetreat(Drivetrain drivetrain, Shooter shooter, ShooterAdjuster adjuster, Storage storage, IntakePlacer intakePlacer) {
        addCommands(
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new Shoot(shooter, drivetrain, adjuster, storage),
                new DriveSwerve(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false).withTimeout(MOVE_TIMEOUT)
        );
    }
}
