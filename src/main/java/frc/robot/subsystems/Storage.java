package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    public Storage(String name, CANSparkMax master) {
        super(name, master);
    }

    public boolean noteInStorage() {
        return false;
    }
}
