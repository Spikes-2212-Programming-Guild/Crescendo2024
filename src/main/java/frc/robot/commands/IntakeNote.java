package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeNote extends ParallelRaceGroup {

    private static final Supplier<Double> ROLLER_SPEED = () -> 0.0;
    private static final Supplier<Double> STORAGE_SPEED = () -> 0.0;

    private final Storage storage;

    public IntakeNote(IntakeRoller intakeRoller, Storage storage) {
        this.storage = storage;
        addCommands(
                new MoveSmartMotorControllerGenericSubsystem(intakeRoller, null, null,
                        UnifiedControlMode.PERCENT_OUTPUT, ROLLER_SPEED),
                new MoveSmartMotorControllerGenericSubsystem(storage, null, null,
                        UnifiedControlMode.PERCENT_OUTPUT, STORAGE_SPEED).until(storage::noteInStorage)
        );
    }
}
