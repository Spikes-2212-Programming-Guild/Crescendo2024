package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.IntakePlacer;

import java.util.function.Supplier;

public class OpenIntake extends MoveSmartMotorControllerGenericSubsystem {

    private static final Supplier<Double> SETPOINT = () -> 0.0;

    public OpenIntake(IntakePlacer intakePlacer) {
        super(intakePlacer, intakePlacer.getPIDSettings(), intakePlacer.getFeedForwardSettings(),
                UnifiedControlMode.POSITION, SETPOINT);
    }
}
