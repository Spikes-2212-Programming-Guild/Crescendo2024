package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap;
import frc.robot.commands.Adjust;

import java.util.function.Supplier;

/**
 * A class which controls the shooter's angle, using one NEO motor.
 */
public class ShooterAdjuster extends SparkGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter adjuster";

    private static final int PID_SLOT = 0;

    private static final int STALL_CURRENT = 30;
    private static final int CURRENT_LIMIT = 20;
    private static final int SECONDARY_CURRENT_LIMIT = 10;
    private static final double MAX_POSITION = 25;

    private static final int SECONDS_IN_MINUTE = 60;

    private static final double RESET_SPEED = 0.25;

    //screw height in cm
    private static final double MOTOR_ROTATIONS_TO_SCREW_HEIGHT = 1 / 6.0;

    private final PIDSettings pidSettings = namespace.addPIDNamespace("", new PIDSettings(0.3, 0.1, 0.05));
    private final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("",
            new FeedForwardSettings(0.9, 0));
    private final FeedForwardSettings resetFFSettings = namespace.addFeedForwardNamespace("reset",
            new FeedForwardSettings(0.25, 0, 0));

    private boolean reset = false;

    private static ShooterAdjuster instance;

    public static ShooterAdjuster getInstance() {
        if (instance == null) {
            instance = new ShooterAdjuster(
                    new CANSparkMax(RobotMap.CAN.SHOOTER_ADJUSTER_SPARK_MAX,
                            CANSparkBase.MotorType.kBrushless));
        }
        return instance;
    }

    private ShooterAdjuster(CANSparkMax motorController) {
        super(NAMESPACE_NAME, motorController);
        master.getEncoder().setPositionConversionFactor(MOTOR_ROTATIONS_TO_SCREW_HEIGHT);
        master.getEncoder().setVelocityConversionFactor(MOTOR_ROTATIONS_TO_SCREW_HEIGHT / SECONDS_IN_MINUTE);
        master.setInverted(true);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        master.getEncoder().setPositionConversionFactor(MOTOR_ROTATIONS_TO_SCREW_HEIGHT);
        master.getEncoder().setVelocityConversionFactor(MOTOR_ROTATIONS_TO_SCREW_HEIGHT / SECONDS_IN_MINUTE);
        master.setInverted(true);
        master.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        master.getPIDController().setOutputRange(-0.7, 0.7);
        configureDashboard();
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType(), PID_SLOT,
                feedForwardSettings.getkS() * Math.signum(setpoint - getPosition()),
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public PIDSettings getPIDSettings() {
        return pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    // since the subsystem did not have any sensors, we created this method which moves the shooter up until it
    // surpasses its current limit, in other words it has difficulties moving. the shooter would usually face those
    // difficulties when it reaches the screw's top, and that way we know that its at the height limit.
    // this command is supposed to run before using any command which uses the adjuster.
    public Command getResetCommand() {
        return new FunctionalCommand(() -> {
        }, () -> master.set(RESET_SPEED), b -> {
            stop();
            reset = true;
            master.getEncoder().setPosition(MAX_POSITION);
        }, () -> Math.abs(getCurrent()) >= STALL_CURRENT, this).andThen(
                new RunCommand(() -> master.getEncoder().setPosition(MAX_POSITION)).withTimeout(0.004));
    }

    public double getPosition() {
        return master.getEncoder().getPosition();
    }

    public double getVelocity() {
        return master.getEncoder().getVelocity();
    }

    public double getCurrent() {
        return master.getOutputCurrent();
    }

    public boolean wasReset() {
        return reset;
    }

    public void set(double speed) {
        master.set(speed);
    }

    public void stop() {
        master.stopMotor();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("position", this::getPosition);
        namespace.putNumber("current", this::getCurrent);
        Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
        Command reset = getResetCommand();
        namespace.putCommand("reset", reset);
        namespace.putBoolean("reset running", reset::isScheduled);
        namespace.putNumber("velocity", this::getVelocity);
        namespace.putRunnable("set position", () -> master.getEncoder().setPosition(setpoint.get()));
        namespace.putNumber("soft limit forward", () -> master.getSoftLimit(CANSparkBase.SoftLimitDirection.kForward));
        namespace.putNumber("soft limit reverse", () -> master.getSoftLimit(CANSparkBase.SoftLimitDirection.kReverse));
        namespace.putNumber("current", master::getOutputCurrent);
        namespace.putCommand("move :sparkles:", new MoveSmartMotorControllerGenericSubsystem(this, pidSettings,
                feedForwardSettings, UnifiedControlMode.POSITION, setpoint));
        namespace.putCommand("new adjust", new Adjust(this, setpoint));
        namespace.putBoolean("was reset", this.wasReset());
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
