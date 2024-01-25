package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class MoveShooter extends Command {
    /**
     * Constructs a new instance of {@link SparkGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the motor controller which runs the loops
     * @param slaves        additional motor controllers that follow the master
     */
    protected final PIDSettings leftPIDSettings;

    /**
     * The PID Settings for the PID loop operating on the right side of the drivetrain.
     */
    protected final PIDSettings rightPIDSettings;

    /**
     * The FeedForwards Settings of the FeedForward loop operating on the left side of the drivetrain.
     */
    protected final FeedForwardSettings leftFeedForwardSettings;

    /**
     * The FeedForwards Settings of the FeedForward loop operating on the right side of the drivetrain.
     */
    protected final FeedForwardSettings rightFeedForwardSettings;
    private final CANSparkBase slave;
    private final CANSparkBase master;

    /**
     * The PID Controller of the PID loop operating on the left side of the drivetrain.
     */
    protected PIDController leftPIDController;

    /**
     * The PID Controller of the PID loop operating on the right side of the drivetrain.
     */
    protected PIDController rightPIDController;

    /**
     * The setpoint the left side of the drivetrain should reach.
     */
    protected Supplier<Double> leftSetpoint;

    /**
     * The setpoint the right side of the drivetrain should reach.
     */
    protected Supplier<Double> rightSetpoint;

    /**
     * How far the left side of the drivetrain drove.
     */
    protected Supplier<Double> leftSource;

    /**
     * How far the right side of the drivetrain drove.
     */
    protected Supplier<Double> rightSource;

    /**
     * The FeedForwards Controller of the FeedForward loop operating on the left side of the drivetrain.
     */
    protected FeedForwardController leftFeedForwardController;

    /**
     * The FeedForwards Controller of the FeedForward loop operating on the right side of the drivetrain.
     */
    protected FeedForwardController rightFeedForwardController;

    public MoveShooter(String namespaceName, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings leftFeedForwardSettings,
                       FeedForwardSettings rightFeedForwardSettings, Supplier<Double> leftSetpoint,
                       Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                       Supplier<Double> rightSource, CANSparkBase master, CANSparkBase slave) {
        this.slave = slave;
        this.leftPIDSettings = leftPIDSettings;
        this.leftPIDController = new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(),
                leftPIDSettings.getkD());
        this.rightPIDSettings = rightPIDSettings;
        this.rightPIDController = new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(),
                rightPIDSettings.getkD());
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftSource = leftSource;
        this.rightSource = rightSource;
        this.leftPIDController.setSetpoint(leftSetpoint.get());
        this.rightPIDController.setSetpoint(rightSetpoint.get());
        this.leftFeedForwardSettings = leftFeedForwardSettings;
        this.rightFeedForwardSettings = rightFeedForwardSettings;
        this.leftFeedForwardController = new FeedForwardController(leftFeedForwardSettings.getkS(),
                leftFeedForwardSettings.getkV(), leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG(),
                FeedForwardController.DEFAULT_PERIOD);
        this.rightFeedForwardController = new FeedForwardController(rightFeedForwardSettings.getkS(),
                rightFeedForwardSettings.getkV(), rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG(),
                FeedForwardController.DEFAULT_PERIOD);
        this.master = master;
    }
    public void execute() {
        leftPIDController.setSetpoint(leftSetpoint.get());
        rightPIDController.setSetpoint(rightSetpoint.get());
        leftPIDController.setTolerance(leftPIDSettings.getTolerance());
        rightPIDController.setTolerance(rightPIDSettings.getTolerance());
        leftPIDController.setPID(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
        rightPIDController.setPID(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
        leftFeedForwardController.setGains(leftFeedForwardSettings.getkS(), leftFeedForwardSettings.getkV(),
                leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG());
        rightFeedForwardController.setGains(rightFeedForwardSettings.getkS(), rightFeedForwardSettings.getkV(),
                rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG());
        master.set(leftPIDController.calculate(leftSource.get()) +
                        leftFeedForwardController.calculate(leftSetpoint.get()));
        slave.set(rightPIDController.calculate(rightSource.get()) +
                rightFeedForwardController.calculate(rightSetpoint.get()));
    }


}
