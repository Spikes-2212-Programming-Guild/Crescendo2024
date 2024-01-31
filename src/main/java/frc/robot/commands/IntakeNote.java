package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePlacer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeNote extends SequentialCommandGroup {

    private static final Supplier<Double> ROLLER_SPEED = () -> 0.0;
    private static final Supplier<Double> STORAGE_SPEED = () -> 0.0;

    public IntakeNote(IntakeRoller intakeRoller, Storage storage, IntakePlacer intakePlacer,
                      boolean moveIntakeAutomatically) {
        if (!storage.hasNote() && moveIntakeAutomatically) {
            addCommands(
                    new OpenIntake(intakePlacer),
                    new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED).raceWith(
                            new MoveGenericSubsystem(storage, STORAGE_SPEED)),
                    new CloseIntake(intakePlacer)
            );
        } else {
            addCommands(new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED).raceWith(
                    new MoveGenericSubsystem(storage, STORAGE_SPEED)));
        }
    }
}
