package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class IntakePlacer extends DashboardedSubsystem implements SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "intake placer";
    private static final String LEFT_NAMESPACE_NAME = "left";
    private static final String RIGHT_NAMESPACE_NAME = "right";

    private final Supplier<Double> VAL = namespace.addConstantDouble("speed", 0);

    private final PIDSettings pidSettings = namespace.addPIDNamespace("", PIDSettings.EMPTY_PID_SETTINGS);
    private final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("",
            FeedForwardSettings.EMPTY_FFSETTINGS);

    private final SparkGenericSubsystem left;
    private final SparkGenericSubsystem right;

    private final CANSparkMax leftSparkMax;
    private final CANSparkMax rightSparkMax;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private static IntakePlacer instance;

    public static IntakePlacer getInstance() {
        if (instance == null) {
            instance = new IntakePlacer(
                    new CANSparkMax(RobotMap.CAN.INTAKE_PLACER_LEFT_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.INTAKE_PLACER_RIGHT_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.DIO.INTAKE_PLACER_TOP_LIMIT_SWITCH),
                    new DigitalInput(RobotMap.DIO.INTAKE_PLACER_BOTTOM_LIMIT_SWITCH));
        }
        return instance;
    }

    private IntakePlacer(CANSparkMax left, CANSparkMax right, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
        super(NAMESPACE_NAME);
        this.left = new SparkGenericSubsystem(LEFT_NAMESPACE_NAME, left);
        this.right = new SparkGenericSubsystem(RIGHT_NAMESPACE_NAME, right);
        this.leftSparkMax = left;
        this.rightSparkMax = right;
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
        leftSparkMax.restoreFactoryDefaults();
        rightSparkMax.restoreFactoryDefaults();
        left.setInverted(false);
        leftSparkMax.setInverted(false);
        right.setInverted(false);
        rightSparkMax.setInverted(false);
        left.setIdleMode(CANSparkBase.IdleMode.kCoast);
        right.setIdleMode(CANSparkBase.IdleMode.kCoast);
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

    public void move() {
        leftSparkMax.set(VAL.get());
        rightSparkMax.set(-VAL.get());
    }

    public void stop() {
        leftSparkMax.stopMotor();
        rightSparkMax.stopMotor();
    }

    @Override
    public void configureDashboard() {
        namespace.putRunnable("nyoom", this::move);
        namespace.putRunnable("stop", this::stop);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        pidSet(controlMode, setpoint, pidSettings, feedForwardSettings, trapezoidProfileSettings);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings) {
        left.pidSet(controlMode, setpoint, pidSettings, feedForwardSettings);
        right.pidSet(controlMode, setpoint, pidSettings, feedForwardSettings);
    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        return left.onTarget(controlMode, tolerance, setpoint) && right.onTarget(controlMode, tolerance, setpoint);
    }
}
