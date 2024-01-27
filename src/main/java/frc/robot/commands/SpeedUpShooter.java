package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class SpeedUpShooter extends Command {
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

    SparkGenericSubsystem leftMotor;
    SparkGenericSubsystem rightMotor;
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

    public SpeedUpShooter(String namespaceName, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                          FeedForwardSettings leftFeedForwardSettings,
                          FeedForwardSettings rightFeedForwardSettings, Supplier<Double> leftSetpoint,
                          Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                          Supplier<Double> rightSource, CANSparkBase lMotor, CANSparkBase rMotor) {
        this.leftPIDSettings = leftPIDSettings;
        this.rightPIDSettings = rightPIDSettings;
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftSource = leftSource;
        this.rightSource = rightSource;
        this.leftFeedForwardSettings = leftFeedForwardSettings;
        this.rightFeedForwardSettings = rightFeedForwardSettings;
        this.leftFeedForwardController = new FeedForwardController(leftFeedForwardSettings.getkS(),
                leftFeedForwardSettings.getkV(), leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG(),
                FeedForwardController.DEFAULT_PERIOD);
        this.rightFeedForwardController = new FeedForwardController(rightFeedForwardSettings.getkS(),
                rightFeedForwardSettings.getkV(), rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG(),
                FeedForwardController.DEFAULT_PERIOD);
        this.leftMotor = new  SparkGenericSubsystem("leftMotor", lMotor, (CANSparkBase) null);
        this.rightMotor = new  SparkGenericSubsystem("leftMotor", rMotor, (CANSparkBase) null);
        leftMotor.configPIDF(leftPIDSettings,leftFeedForwardSettings);
        rightMotor.configPIDF(rightPIDSettings,rightFeedForwardSettings);
    }

    @Override
    public void execute() {
        leftMotor.pidSet(UnifiedControlMode.VELOCITY,leftSetpoint.get(),leftPIDSettings,leftFeedForwardSettings);
        rightMotor.pidSet(UnifiedControlMode.VELOCITY,rightSetpoint.get(),rightPIDSettings,rightFeedForwardSettings);
    }
    public void stop(){

    }

}

