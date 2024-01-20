package frc.robot.commands;

import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class RotateWithPID extends Command {

    private final Drivetrain drivetrain;
    private final Supplier<Double> targetAngle;
    private final Supplier<Double> sourceAngle;
    private final PIDSettings pidSettings;
    private final PIDController pidController;

    private double lastTimeNotOnTarget;

    public RotateWithPID(Drivetrain drivetrain, Supplier<Double> targetAngle, Supplier<Double> sourceAngle,
                         PIDSettings pidSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        this.targetAngle = targetAngle;
        this.sourceAngle = sourceAngle;
        this.pidSettings = pidSettings;
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
    }

    @Override
    public void execute() {
        drivetrain.drive(0, 0, pidController.calculate(targetAngle.get(), sourceAngle.get()), false, false);
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
