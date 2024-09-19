package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePlacer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;
import frc.robot.util.LEDService;

import java.util.function.Supplier;

public class IntakeNote extends SequentialCommandGroup {

    private final RootNamespace root = new RootNamespace("intake note");

    private static final Supplier<Double> ROLLER_SPEED = () -> -0.85;
    private static final double STORAGE_VOLTAGE = -4.0;
    public static final Supplier<Double> SHOOTER_HEIGHT = () -> 8.0;

    private final LEDService ledService;

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
        ledService = LEDService.getInstance();
            addCommands(
                    new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                    adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, SHOOTER_HEIGHT) {
                        @Override
                        public void execute() {
                            ledService.attemptIntake();
                            super.execute();
                        }

                        @Override
                        public boolean isFinished() {
                            return super.isFinished() || !adjuster.wasReset();
                        }
                    },
                    new MoveGenericSubsystem(intakeRoller, ROLLER_SPEED)
                            .raceWith(new MoveGenericSubsystem(storage, () -> STORAGE_VOLTAGE / RobotController.getBatteryVoltage())),
                    new InstantCommand(ledService::intakeSuccessful));
//            addCommands(nezw MoveGenericSubsystem(intakeRoller, 0.5)
//                    .alongWith(new MoveGenericSubsystem(storage, 0.5)).until(storage::hasNote)
//                    .andThen(new InstantCommand(storage::stop).alongWith(new InstantCommand(intakeRoller::stop))));
//        }
    }
}
