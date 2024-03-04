package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

public class JustShoot extends SequentialCommandGroup {

    public JustShoot(Shooter shooter, ShooterAdjuster adjuster, IntakePlacer intakePlacer, Drivetrain drivetrain,
                     Storage storage) {
        addCommands(
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new Shoot(shooter, drivetrain, adjuster, storage, Shoot.CLOSE_HEIGHT)
        );
    }
}
