package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

/**
 * Command which rotates the drivetrain to a certain angle using PID + FF control.
 */
public class RotateSwerveWithPID extends Command {

    private final Drivetrain drivetrain;
    private final Supplier<Double> setpoint;
    private final Supplier<Double> source;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final PIDSettings pidSettings;
    private final PIDController pidController;
    private final FeedForwardSettings feedForwardSettings;
    private final FeedForwardController feedForwardController;

    private double lastTimeNotOnTarget;

    public RotateSwerveWithPID(Drivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> source,
                         Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                         PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.source = source;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.pidSettings = pidSettings;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        this.feedForwardSettings = feedForwardSettings;
        this.feedForwardController = new FeedForwardController(feedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
    }

    public RotateSwerveWithPID(Drivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> source,
                               PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        this(drivetrain, setpoint, source, () -> 0.0, () -> 0.0, pidSettings, feedForwardSettings);
    }

    @Override
    public void initialize() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
    }

    @Override
    public void execute() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        feedForwardController.setGains(feedForwardSettings);
        double kS = feedForwardSettings.getkS();
        feedForwardController.setkS(0);
        drivetrain.drive(xSpeed.get(), ySpeed.get(), pidController.calculate(source.get(),
                setpoint.get()) + feedForwardController.calculate(setpoint.get())
                + kS * Math.signum(setpoint.get() - source.get()), false, false, false);
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
