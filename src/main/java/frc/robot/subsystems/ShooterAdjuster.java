package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

public class ShooterAdjuster extends SparkGenericSubsystem {

    private static String NAMESPACE_NAME = "shooter adjuster";
    private static final double ENCODER_OFFSET = 0;

    private final DutyCycleEncoder absoluteEncoder;

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private static ShooterAdjuster instance;

    public static ShooterAdjuster getInstance() {
        if (instance == null) {
            instance = new ShooterAdjuster(
                    new CANSparkMax(RobotMap.CAN.SHOOTER_ADJUSTER_LEFT_SPARKMAX,
                            CANSparkBase.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.SHOOTER_ADJUSTER_RIGHT_SPARKMAX,
                            CANSparkBase.MotorType.kBrushless),
                    new DutyCycleEncoder(RobotMap.DIO.SHOOTER_ADJUSTER_ABSOLUTE_ENCODER),
                    new DigitalInput(RobotMap.DIO.SHOOTER_ADJUSTER_TOP_LIMIT),
                    new DigitalInput(RobotMap.DIO.SHOOTER_ADJUSTER_BOTTOM_LIMIT));
        }
        return instance;
    }

    private ShooterAdjuster(CANSparkMax left, CANSparkMax right, DutyCycleEncoder absoluteEncoder,
                            DigitalInput topLimit, DigitalInput bottomLimit) {
        super(NAMESPACE_NAME, left, right);
        this.absoluteEncoder = absoluteEncoder;
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
    }

    public double getAngle() {
        return absoluteEncoder.get() * 360 + ENCODER_OFFSET;
    }
}
