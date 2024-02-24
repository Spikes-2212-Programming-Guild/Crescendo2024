package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.ChildNamespace;
import com.spikes2212.dashboard.SpikesLogger;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.commands.Adjust;

import java.util.function.Supplier;

public class ShooterAdjuster extends SparkGenericSubsystem {

    public static final double CURRENT_LIMIT = 40;
    private static final double RESET_SPEED = 0.2;
    private static final double MAX_POSITION = 25;

    private static final String NAMESPACE_NAME = "shooter adjuster";
    private static final double ENCODER_OFFSET = 0;
    //screw height in cm
    private static final double MOTOR_ROTATIONS_TO_SCREW_HEIGHT = 1 / 6.0;

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

    private final DigitalInput topHallEffect;
    private final DigitalInput bottomLimit;

    private boolean reset = false;

    private static ShooterAdjuster instance;

    public static ShooterAdjuster getInstance() {
        if (instance == null) {
            instance = new ShooterAdjuster(
                    new CANSparkMax(RobotMap.CAN.SHOOTER_ADJUSTER_SPARK_MAX,
                            CANSparkBase.MotorType.kBrushless),
                    new DutyCycleEncoder(RobotMap.DIO.SHOOTER_ADJUSTER_ABSOLUTE_ENCODER),
                    new DigitalInput(RobotMap.DIO.SHOOTER_ADJUSTER_TOP_HALL_EFFECT),
                    new DigitalInput(RobotMap.DIO.SHOOTER_ADJUSTER_BOTTOM_LIMIT));
        }
        return instance;
    }

    private ShooterAdjuster(CANSparkMax motorController, DutyCycleEncoder absoluteEncoder,
                            DigitalInput topHallEffect, DigitalInput bottomLimit) {
        super(NAMESPACE_NAME, motorController);
        this.absoluteEncoder = absoluteEncoder;
        this.topHallEffect = topHallEffect;
        this.bottomLimit = bottomLimit;
//        master.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 24);
//        master.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 10);
        master.setInverted(true);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        master.getEncoder().setPositionConversionFactor(MOTOR_ROTATIONS_TO_SCREW_HEIGHT);
        master.getEncoder().setVelocityConversionFactor(MOTOR_ROTATIONS_TO_SCREW_HEIGHT / 60);
        master.setInverted(true);
//        master.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 22);
//        master.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 10);
        master.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
        master.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false);
        configureDashboard();
//        master.setSmartCurrentLimit(50);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType(), 0,
                feedForwardSettings.getkS() * Math.signum(setpoint - getPosition()), SparkPIDController.ArbFFUnits.kVoltage);
    }

    //in cm
    public double getScrewHeight() {
        return absoluteEncoder.get() * MOTOR_ROTATIONS_TO_SCREW_HEIGHT + ENCODER_OFFSET;
    }

    public boolean isFullyUp() {
        return !topHallEffect.get();
    }

    public boolean isFullyDown() {
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

    public CANSparkMax getMotor() {
        return (CANSparkMax) master;
    }

    public Command getResetCommand() {
        return new MoveSmartMotorControllerGenericSubsystem(this, new PIDSettings(0, 0, 10000), feedForwardSettings,
                UnifiedControlMode.PERCENT_OUTPUT, () -> RESET_SPEED).until(
                        () -> Math.abs(master.getOutputCurrent()) >= CURRENT_LIMIT)
                .andThen(new InstantCommand(() -> master.getEncoder().setPosition(MAX_POSITION)));
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

    public void move() {
        master.setVoltage(-4);
    }

    public void unmove() {
        if (getPosition() < 22.2) master.setVoltage(4);
        else master.stopMotor();
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
        namespace.putNumber("absolute encoder reading", absoluteEncoder::getAbsolutePosition);
        namespace.putNumber("position", this::getPosition);
        namespace.putNumber("current", master::getOutputCurrent);
        Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
        namespace.putCommand("move",
                new MoveSmartMotorControllerGenericSubsystem(this, pidSettings, feedForwardSettings,
                        UnifiedControlMode.VELOCITY, setpoint));
        namespace.putRunnable("unmove", this::unmove);
        namespace.putCommand("reset", this.getResetCommand().andThen(new InstantCommand(() -> reset = true)));
        namespace.putNumber("velocity", this::getVelocity);
        namespace.putRunnable("set position", () -> master.getEncoder().setPosition(setpoint.get()));
        namespace.putNumber("soft limit forward", () -> master.getSoftLimit(CANSparkBase.SoftLimitDirection.kForward));
        namespace.putNumber("soft limit reverse", () -> master.getSoftLimit(CANSparkBase.SoftLimitDirection.kReverse));
        namespace.putNumber("current", master::getOutputCurrent);
        namespace.putCommand("move :sparkles:", new MoveSmartMotorControllerGenericSubsystem(this, pidSettings,
                feedForwardSettings, UnifiedControlMode.POSITION, setpoint));
        namespace.putCommand("new adjust", new Adjust(this, setpoint));
    }

    @Override
    public void periodic() {
        super.periodic();
        new SpikesLogger().log(getVelocity());
    }
}
