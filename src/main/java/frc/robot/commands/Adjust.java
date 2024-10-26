package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.*;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterAdjuster;

import java.util.function.Supplier;

/**
 * This command first runs the adjuster towards the setpoint by setting the motor's voltage, and once its close it runs
 * a PID control loop.
 */
public class Adjust extends SequentialCommandGroup {

    private static final double PID_START_POINT = 2.5;
    private static final double SPEED = 0.75;

    public final RootNamespace namespace = new RootNamespace("adjust");

    public Adjust(ShooterAdjuster adjuster, Supplier<Double> setpoint) {
        addRequirements(adjuster);

        if (Math.abs(adjuster.getPosition() - setpoint.get()) > PID_START_POINT) {
            addCommands(new FunctionalCommand(() -> {
                    },
                            () -> adjuster.set(SPEED * Math.signum(adjuster.getPosition() - setpoint.get())),
                            interrupted -> adjuster.stop(),
                            () -> !(Math.abs(adjuster.getPosition() - setpoint.get()) > PID_START_POINT)),
                    new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                            adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, setpoint));
        } else addCommands(new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, setpoint));
        configureDashboard();
    }

    public void configureDashboard() {
        namespace.putBoolean("finished", this::isFinished);
    }
}
