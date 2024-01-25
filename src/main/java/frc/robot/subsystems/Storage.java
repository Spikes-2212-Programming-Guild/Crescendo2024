package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private static Storage instance;

    private final DigitalInput limit;

    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage(
                    NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.STORAGE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.CAN.STORAGE_SPARK_MAX)
            );
        }
        return instance;
    }

    private Storage(String namespaceName, MotorController motor, DigitalInput limit) {
        super(namespaceName, motor);
        this.limit = limit;
    }

    public boolean hasNote() {
        return limit.get();
    }
}
