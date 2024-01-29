package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class IntakePlacer extends SparkGenericSubsystem {

    private static final String NAMESPACE_NAME = "intake placer";

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private static IntakePlacer instance;

    public static IntakePlacer getInstance() {
        return instance;
    }

    private IntakePlacer(CANSparkMax motor, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
        super(NAMESPACE_NAME, motor);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }
  
     public boolean intakeUp() {
        return topLimitSwitch.get();
    }

    public boolean intakeDown() {
        return bottomLimitSwitch.get();
    }
}
