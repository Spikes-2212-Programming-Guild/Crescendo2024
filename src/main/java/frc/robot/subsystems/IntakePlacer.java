package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class IntakePlacer extends SparkGenericSubsystem {

    private static final String NAMESPACE_NAME = "intake placer";

    private final PIDSettings pidSettings = namespace.addPIDNamespace("", PIDSettings.EMPTY_PID_SETTINGS);
    private final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("",
            FeedForwardSettings.EMPTY_FFSETTINGS);

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private static IntakePlacer instance;

    public static IntakePlacer getInstance() {
        if (instance == null) {
            instance = new IntakePlacer(
                    new CANSparkMax(RobotMap.CAN.INTAKE_PLACER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.INTAKE_PLACER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.DIO.INTAKE_PLACER_TOP_LIMIT_SWITCH),
                    new DigitalInput(RobotMap.DIO.INTAKE_PLACER_BOTTOM_LIMIT_SWITCH));
        }
        return instance;
    }

    private IntakePlacer(CANSparkMax master, CANSparkMax slave, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
        super(NAMESPACE_NAME, master, slave);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }
  
     public boolean intakeUp() {
        return topLimitSwitch.get();
    }

    public boolean intakeDown() {
        return bottomLimitSwitch.get();
    }

    public PIDSettings getPIDSettings() {
        return pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }
}
