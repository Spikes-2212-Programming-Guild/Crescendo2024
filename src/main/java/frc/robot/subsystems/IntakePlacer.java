package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

public class IntakePlacer extends SparkGenericSubsystem {

    public IntakePlacer(String namespaceName, CANSparkBase master, CANSparkBase... slaves) {
        super(namespaceName, master, slaves);
    }

    public PIDSettings getPIDSettings() {
        return null;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return null;
    }

    public boolean intakeUp() {
        return true;
    }

    public boolean intakeDown() {
        return true;
    }
}
