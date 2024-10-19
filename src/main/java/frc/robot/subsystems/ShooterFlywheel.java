package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.RobotMap;

import java.util.function.Supplier;

/**
 * A class which represents a single shooter flywheel, which uses 1 NEO.
 */
public class ShooterFlywheel extends SparkGenericSubsystem {

    private static final PIDSettings LEFT_PID_SETTINGS = new PIDSettings(0.0001, 75, 10000000);
    private static final FeedForwardSettings LEFT_FF_SETTINGS = new FeedForwardSettings(0.69, 0.00214, 0);
    private static final PIDSettings RIGHT_PID_SETTINGS = new PIDSettings(0.0001, 75, 10000000);
    private static final FeedForwardSettings RIGHT_FF_SETTINGS = new FeedForwardSettings(0.52, 0.00213, 0);

    public final PIDSettings pidSettings = namespace.addPIDNamespace("", new PIDSettings(0, 0, 100000000));
    public final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("",
            FeedForwardSettings.EMPTY_FFSETTINGS);

    private final FeedForwardController feedForwardController;

    private final boolean inverted;

    private final CANSparkMax sparkMax;

    private static ShooterFlywheel leftInstance;
    private static ShooterFlywheel rightInstance;

    public static ShooterFlywheel getLeftInstance() {
        if (leftInstance == null) {
            leftInstance = new ShooterFlywheel("left flywheel",
                    new CANSparkMax(RobotMap.CAN.LEFT_SHOOTER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless), false);
        }
        return leftInstance;
    }

    public static ShooterFlywheel getRightInstance() {
        if (rightInstance == null) {
            rightInstance = new ShooterFlywheel("right flywheel",
                    new CANSparkMax(RobotMap.CAN.RIGHT_SHOOTER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless), true);
        }
        return rightInstance;
    }

    public ShooterFlywheel(String namespaceName, CANSparkMax master, boolean inverted) {
        super(namespaceName, master);
        feedForwardController = new FeedForwardController(feedForwardSettings, FeedForwardController.DEFAULT_PERIOD);
        this.inverted = inverted;
        this.sparkMax = master;
        master.setInverted(inverted);
        configureDashboard();
    }

    @Override
    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        master.getPIDController().setP(pidSettings.getkP());
        master.getPIDController().setI(pidSettings.getkI());
        master.getPIDController().setD(pidSettings.getkD());
        feedForwardController.setGains(feedForwardSettings);
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        master.setInverted(inverted);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType(), 0,
                feedForwardController.calculate(setpoint), SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("speed", () -> master.getEncoder().getVelocity());
        Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
        namespace.putCommand("move", new MoveSmartMotorControllerGenericSubsystem(this, pidSettings,
                feedForwardSettings, UnifiedControlMode.VELOCITY, setpoint));
        namespace.putCommand("test", new MoveSmartMotorControllerGenericSubsystem(this, pidSettings,
                feedForwardSettings, UnifiedControlMode.VELOCITY, () -> 800.0));
    }

    public void stop() {
        sparkMax.stopMotor();
    }
}
