package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveSwerve extends Command {

    private static final double DRIVE_ACCELERATION_LIMIT = 2;
    private static final double TURN_ACCELERATION_LIMIT_PERCENTAGE = 4;

    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;
    private final boolean fieldRelative;
    private final boolean usePID;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotationLimiter;

    public DriveSwerve(Drivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed, boolean fieldRelative, boolean usePID) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.fieldRelative = fieldRelative;
        this.usePID = usePID;
        this.xLimiter = new SlewRateLimiter(DRIVE_ACCELERATION_LIMIT);
        this.yLimiter = new SlewRateLimiter(DRIVE_ACCELERATION_LIMIT);
        this.rotationLimiter = new SlewRateLimiter(TURN_ACCELERATION_LIMIT_PERCENTAGE);
    }

    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(this.xSpeed.get());
        double ySpeed = yLimiter.calculate(this.ySpeed.get());
        double rotationSpeed = rotationLimiter.calculate(this.rotationSpeed.get());

        if (Math.abs(xSpeed) < Drivetrain.MIN_SPEED_METERS_PER_SECONDS) xSpeed = 0;
        if (Math.abs(ySpeed) < Drivetrain.MIN_SPEED_METERS_PER_SECONDS) ySpeed = 0;
        if (Math.abs(rotationSpeed) < Drivetrain.MIN_SPEED_METERS_PER_SECONDS) rotationSpeed = 0;

        drivetrain.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
