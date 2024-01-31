package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import frc.robot.RobotMap;

public class Shooter extends DashboardedSubsystem {

    private static final String NAMESPACE_NAME = "shooter";
    private static final String LEFT_FLYWHEEL_NAMESPACE_NAME = "left flywheel";
    private static final String RIGHT_FLYWHEEL_NAMESPACE_NAME = "right flywheel";

    public final PIDSettings leftPIDSettings = namespace.addPIDNamespace("left", PIDSettings.EMPTY_PID_SETTINGS);
    public final PIDSettings rightPIDSettings = namespace.addPIDNamespace("right", PIDSettings.EMPTY_PID_SETTINGS);
    public final FeedForwardSettings leftFeedForwardSettings =
            namespace.addFeedForwardNamespace("left", FeedForwardSettings.EMPTY_FFSETTINGS);
    public final FeedForwardSettings rightFeedForwardSettings =
            namespace.addFeedForwardNamespace("right", FeedForwardSettings.EMPTY_FFSETTINGS);

    private final SparkGenericSubsystem leftFlywheel;
    private final SparkGenericSubsystem rightFlywheel;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter(new CANSparkMax(RobotMap.CAN.LEFT_SHOOTER_SPARK_MAX,CANSparkLowLevel.MotorType.kBrushless),
                    new  CANSparkMax(RobotMap.CAN.RIGHT_SHOOTER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless)
            );
        }
        return instance;
    }

    public Shooter(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        super(NAMESPACE_NAME);
        this.leftFlywheel = new SparkGenericSubsystem(LEFT_FLYWHEEL_NAMESPACE_NAME, leftMotor);
        this.rightFlywheel = new SparkGenericSubsystem(RIGHT_FLYWHEEL_NAMESPACE_NAME, rightMotor);
    }

    public SparkGenericSubsystem getLeftFlywheel() {
        return leftFlywheel;
    }

    public SparkGenericSubsystem getRightFlywheel() {
        return rightFlywheel;
    }

    @Override
    public void configureDashboard() {
    }
}
