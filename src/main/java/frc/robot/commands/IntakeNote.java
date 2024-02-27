package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePlacer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeNote extends SequentialCommandGroup {

    private final RootNamespace root = new RootNamespace("intake note");

    private static final Supplier<Double> ROLLER_SPEED = () -> -0.85;
    private static final Supplier<Double> STORAGE_SPEED = () -> -0.25;
    private static final Supplier<Double> SHOOTER_HEIGHT = () -> 6.0;

    public IntakeNote(IntakeRoller intakeRoller, Storage storage, IntakePlacer intakePlacer,
                      ShooterAdjuster adjuster, boolean moveIntakeAutomatically) {
//        if (!storage.seesNote() && moveIntakeAutomatically) {
//            addCommands(
//                    new OpenIntake(intakePlacer),
//                    new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
//                            adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, SHOOTER_HEIGHT),
//                    new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED).raceWith(
//                            new MoveGenericSubsystem(storage, STORAGE_SPEED)),
//                    new CloseIntake(intakePlacer)
//            );
//        } else if (!storage.seesNote()) {
            addCommands(
                    new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                    adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, SHOOTER_HEIGHT),
                    new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED)
                            .raceWith(new MoveGenericSubsystem(storage, STORAGE_SPEED)));
//            addCommands(new MoveGenericSubsystem(intakeRoller, 0.5)
//                    .alongWith(new MoveGenericSubsystem(storage, 0.5)).until(storage::hasNote)
//                    .andThen(new InstantCommand(storage::stop).alongWith(new InstantCommand(intakeRoller::stop))));
//        }
    }
}
