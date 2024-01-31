package frc.robot.commands;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class RotateSwerveWithPID extends Command {

    public RotateSwerveWithPID(Drivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> source,
                               PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {

    }
}
