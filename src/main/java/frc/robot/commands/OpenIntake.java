package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePlacer;

import java.util.function.Supplier;

public class OpenIntake extends SequentialCommandGroup {

    private static final Supplier<Double> SPEED = () -> -0.1;
    private static final double RUNTIME = 0.5;

    public OpenIntake(IntakePlacer intakePlacer) {
        addRequirements(intakePlacer);
        addCommands(new MoveSmartMotorControllerGenericSubsystem(intakePlacer, new PIDSettings(0, 0, 10000),
                FeedForwardSettings.EMPTY_FFSETTINGS, UnifiedControlMode.PERCENT_OUTPUT, SPEED).withTimeout(RUNTIME));
    }
}
