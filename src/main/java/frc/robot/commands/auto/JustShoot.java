package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

/**
 * Auto which shoots the starting note.
 */
public class JustShoot extends SequentialCommandGroup {

    public JustShoot(Shooter shooter, ShooterAdjuster adjuster, IntakePlacer intakePlacer, Drivetrain drivetrain,
                     Storage storage) {
        addCommands(
                new InstantCommand(drivetrain::resetGyro),
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new Shoot(shooter, drivetrain, adjuster, storage, Shoot.CLOSE_HEIGHT)
        );
    }
}
