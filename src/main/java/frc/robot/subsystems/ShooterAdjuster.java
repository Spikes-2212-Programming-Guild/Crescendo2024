package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.ChildNamespace;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class ShooterAdjuster extends SparkGenericSubsystem {

    private static String NAMESPACE_NAME = "shooter adjuster";
    private static final double ENCODER_OFFSET = 0;
    private static final double MOTOR_ROTATIONS_TO_ANGLE_RATIO = 0;

    private final ChildNamespace trapezoidProfileNamespace = namespace.addChild("trapezoid profile");
    private final Supplier<Double> acceleration = trapezoidProfileNamespace.addConstantDouble("acceleration", 0);
    private final Supplier<Double> maxVelocity = trapezoidProfileNamespace.addConstantDouble("max velocity", 0);
    private final Supplier<Integer> curve = trapezoidProfileNamespace.addConstantInt("curve", 0);
    private final PIDSettings pidSettings = namespace.addPIDNamespace("", PIDSettings.EMPTY_PID_SETTINGS);
    private final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("",
            FeedForwardSettings.EMPTY_FFSETTINGS);
    private final TrapezoidProfileSettings trapezoidProfileSettings = new TrapezoidProfileSettings(acceleration,
            maxVelocity, curve);

    private final DutyCycleEncoder absoluteEncoder;

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private static ShooterAdjuster instance;

    public static ShooterAdjuster getInstance() {
        if (instance == null) {
            instance = new ShooterAdjuster(
                    new CANSparkMax(RobotMap.CAN.SHOOTER_ADJUSTER_SPARK_MAX,
                            CANSparkBase.MotorType.kBrushless),
                    new DutyCycleEncoder(RobotMap.DIO.SHOOTER_ADJUSTER_ABSOLUTE_ENCODER),
                    new DigitalInput(RobotMap.DIO.SHOOTER_ADJUSTER_TOP_LIMIT),
                    new DigitalInput(RobotMap.DIO.SHOOTER_ADJUSTER_BOTTOM_LIMIT));
        }
        return instance;
    }

    private ShooterAdjuster(CANSparkMax motorController, DutyCycleEncoder absoluteEncoder,
                            DigitalInput topLimit, DigitalInput bottomLimit) {
        super(NAMESPACE_NAME, motorController);
        this.absoluteEncoder = absoluteEncoder;
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        master.getEncoder().setPositionConversionFactor(MOTOR_ROTATIONS_TO_ANGLE_RATIO);
    }

    public double getAngle() {
        return absoluteEncoder.get() * 360 + ENCODER_OFFSET;
    }

    public boolean topLimitHit() {
        return topLimit.get();
    }

    public boolean bottomLimitHit() {
        return bottomLimit.get();
    }

    public PIDSettings getPIDSettings() {
        return pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return trapezoidProfileSettings;
    }

}
