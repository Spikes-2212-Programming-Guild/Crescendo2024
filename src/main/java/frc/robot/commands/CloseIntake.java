package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.IntakePlacer;

import java.util.function.Supplier;

public class CloseIntake extends MoveSmartMotorControllerGenericSubsystem {

    private static final Supplier<Double> SETPOINT = () -> 0.0;

    public CloseIntake(IntakePlacer intakePlacer) {
        super(intakePlacer, intakePlacer.getPIDSettings(), intakePlacer.getFeedForwardSettings(),
                UnifiedControlMode.POSITION, SETPOINT);
    }

    @Override
    public boolean isFinished() {
       return ((IntakePlacer) subsystem).intakeUp() || super.isFinished();
    }
}
