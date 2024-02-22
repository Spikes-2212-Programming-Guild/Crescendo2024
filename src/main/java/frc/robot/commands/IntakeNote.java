package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePlacer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeNote extends SequentialCommandGroup {

    private static final Supplier<Double> ROLLER_SPEED = () -> 0.75;
    private static final Supplier<Double> STORAGE_SPEED = () -> -0.75;
    private static final Supplier<Double> SHOOTER_HEIGHT = () -> 10.0;

    public IntakeNote(IntakeRoller intakeRoller, Storage storage, IntakePlacer intakePlacer,
                      ShooterAdjuster adjuster, boolean moveIntakeAutomatically) {
        if (!storage.hasNote() && moveIntakeAutomatically) {
            addCommands(
                    new OpenIntake(intakePlacer),
                    new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                            adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, SHOOTER_HEIGHT),
                    new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED).raceWith(
                            new MoveGenericSubsystem(storage, STORAGE_SPEED)),
                    new CloseIntake(intakePlacer)
            );
        } else if (!storage.hasNote()) {
            addCommands(new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                    adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, SHOOTER_HEIGHT),
                    new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED) {

                        private static final double MIN_TIME  = 0.5;
                        private double start;

                        @Override
                        public void initialize() {
                            start = Timer.getFPGATimestamp();
                        }

                        @Override
                        public boolean isFinished() {
//                            return storage.getCurrent() >= 20
//                                    && Timer.getFPGATimestamp() - start >= MIN_TIME;
                            return false;
                        }
                    }.raceWith(new MoveGenericSubsystem(storage, STORAGE_SPEED)));
        }
    }
}
