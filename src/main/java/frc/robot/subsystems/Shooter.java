package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends Command {
    public static CANSparkBase slave;
    private final String namespaceName;
    public final CANSparkBase master;

    public PIDSettings leftPidSettings(){
        return null;
    }
    public PIDSettings rightPidSettings(){
        return null;
    }
    public FeedForwardSettings leftFeedForwardSettings(){
        return null;
    }
    public FeedForwardSettings rightForwardSettings(){
        return null;
    }
    public double getLeftVelocity(){
        return 0.0;
    }
    public double getRightVelocity(){
        return 0.0;
    }

    /**
     * Constructs a new instance of {@link SparkGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the master
     */
    public Shooter(String namespaceName, CANSparkBase master,CANSparkBase slave) {
        this.namespaceName =namespaceName;
        this.master = master;
        this.slave =slave;
    }
}
