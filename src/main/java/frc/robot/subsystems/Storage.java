package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    public Storage(String name, CANSparkMax master) {
        super(name, master);
    }

    public boolean hasNote() {
        return false;
    }
}
