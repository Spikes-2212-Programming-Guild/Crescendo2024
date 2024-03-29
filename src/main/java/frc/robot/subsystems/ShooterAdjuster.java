package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class ShooterAdjuster extends SparkGenericSubsystem {
    /**
     * Constructs a new instance of {@link SparkGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the master
     */
    public ShooterAdjuster(String namespaceName, CANSparkBase master, CANSparkBase... slaves) {
        super(namespaceName, master, slaves);
    }
}
