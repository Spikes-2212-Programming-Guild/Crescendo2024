package frc.robot.command;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class RotateSwerveWithPID extends Command {
    public RotateSwerveWithPID(Drivetrain drivetrain, Supplier<Double> sours, Supplier<Double> setpoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings){

    }
    public RotateSwerveWithPID(Supplier<Double> source ,Supplier<Double> setPoint ){

    }

}
