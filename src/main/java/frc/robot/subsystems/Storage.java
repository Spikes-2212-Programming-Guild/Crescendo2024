package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    public Storage(String namespaceName, CANSparkBase motor) {

        super(namespaceName, motor);

    }
}
