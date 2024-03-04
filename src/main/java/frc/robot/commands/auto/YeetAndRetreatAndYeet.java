package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class YeetAndRetreatAndYeet extends SequentialCommandGroup {

    private static final double MOVE_TIMEOUT = 1.5;
    private static final double DRIVE_SPEED = 1.0;

    public YeetAndRetreatAndYeet(Drivetrain drivetrain, Shooter shooter, ShooterAdjuster adjuster, IntakePlacer intakePlacer,
                                 IntakeRoller intakeRoller, Storage storage) {
        addCommands(
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new ParallelCommandGroup(
                        new Shoot(shooter, drivetrain, adjuster, storage, Shoot.CLOSE_HEIGHT),
                        new OpenIntake(intakePlacer)
                ),
                new ParallelRaceGroup(
                        new DriveSwerve(drivetrain, () -> DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false)
                                .withTimeout(MOVE_TIMEOUT),
                        new IntakeNote(intakeRoller, storage, intakePlacer, adjuster, false)
                ),
                new DriveSwerve(drivetrain, () -> -DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false).withTimeout(MOVE_TIMEOUT),
                new ParallelCommandGroup(
                        new Shoot(shooter, drivetrain, adjuster, storage, Shoot.CLOSE_HEIGHT),
                        new CloseIntake(intakePlacer)
                )
        );
    }
}
