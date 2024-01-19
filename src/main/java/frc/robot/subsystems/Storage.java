package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class Storage extends SparkGenericSubsystem {

    public Storage(String name, CANSparkMax master, CANSparkMax... slaves) {
        super(name, master, slaves);
    }

    public boolean noteInStorage() {
        return false;
    }
}
