package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    private static Storage instance;

    public static Storage getInstance() {
        return instance;
    }

    private Storage(String namespaceName, CANSparkMax motorController) {
        super(namespaceName, motorController);
    }

    public boolean hasNote() {
        return false;
    }
}
