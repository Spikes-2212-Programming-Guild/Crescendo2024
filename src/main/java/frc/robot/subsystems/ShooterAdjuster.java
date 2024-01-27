package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAdjuster extends SparkGenericSubsystem {

    public PIDSettings pidSettings(){
        return null;
    }
    public FeedForwardSettings feedForwardSettings(){
        return null;
    }

    public TrapezoidProfileSettings trapezoidProfileSettings(){
        return null;
    }

    /**
     * Constructs a new instance of {@link SparkGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the leftMotor
     */
    public ShooterAdjuster(String namespaceName, CANSparkBase master, CANSparkBase... slaves) {

        super(namespaceName, master, slaves);
    }

}
