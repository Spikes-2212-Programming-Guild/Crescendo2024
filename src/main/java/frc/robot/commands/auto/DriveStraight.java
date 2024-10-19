package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakePlacer;
import frc.robot.subsystems.ShooterAdjuster;
import java.util.function.Supplier;

/**
 * Auto which drives out of the wing then stops.
 */
public class DriveStraight extends SequentialCommandGroup {

    private static final double MOVE_TIMEOUT = 4;
    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;

    public DriveStraight(Drivetrain drivetrain, ShooterAdjuster adjuster, IntakePlacer intakePlacer) {
        addCommands(
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new DriveSwerve(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false)
                        .withTimeout(MOVE_TIMEOUT)
        );
    }
}
