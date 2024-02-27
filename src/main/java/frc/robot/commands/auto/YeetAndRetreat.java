package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class YeetAndRetreat extends SequentialCommandGroup {

    private static final double MOVE_TIMEOUT = 4;
    private static final Supplier<Double> DRIVE_SPEED = () -> 2.0;

    //Click A-Stop if necessary!
    public YeetAndRetreat(Drivetrain drivetrain, Shooter shooter, ShooterAdjuster adjuster, Storage storage) {
        addCommands(
                adjuster.getResetCommand(),
                new Shoot(shooter, drivetrain, adjuster, storage),
                new DriveSwerve(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false).withTimeout(MOVE_TIMEOUT)
        );
    }
}
