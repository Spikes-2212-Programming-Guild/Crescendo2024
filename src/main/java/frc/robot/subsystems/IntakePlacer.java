package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

public class IntakePlacer extends SparkGenericSubsystem {

    /**
     * Constructs a new instance of {@link SparkGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the master
     */
    public IntakePlacer(String namespaceName, CANSparkBase master, CANSparkBase... slaves) {
        super(namespaceName, master, slaves);
    }

    public PIDSettings getPIDSettings() {
        return null;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return null;
    }
}
