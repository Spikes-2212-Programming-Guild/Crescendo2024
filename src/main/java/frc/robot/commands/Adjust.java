package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterAdjuster;

import java.util.function.Supplier;

public class Adjust extends SequentialCommandGroup {

    public RootNamespace namespace = new RootNamespace("why");

    public Adjust(ShooterAdjuster adjuster, Supplier<Double> setpoint) {
        addRequirements(adjuster);
        namespace.putBoolean("finished", this::isFinished);
        if (Math.abs(adjuster.getPosition() - setpoint.get()) > 2.5) {
            addCommands(new RunCommand(() -> adjuster.set(0.75 * Math.signum(adjuster.getPosition() - setpoint.get()))) {
                            @Override
                            public boolean isFinished() {
                                return Math.abs(adjuster.getPosition() - setpoint.get()) > 2.5;
                            }

                            @Override
                            public void end(boolean interrupted) {
                                adjuster.stop();
                            }
                        },
                    new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                            adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, setpoint));
        } else addCommands(new MoveSmartMotorControllerGenericSubsystem(adjuster, adjuster.getPIDSettings(),
                adjuster.getFeedForwardSettings(), UnifiedControlMode.POSITION, setpoint));
    }
}
