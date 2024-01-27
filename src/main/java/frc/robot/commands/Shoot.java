package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GetFunk;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class Shoot extends ParallelCommandGroup {
    private final GetFunk getFunk;

    public Shoot(frc.robot.subsystems.ShooterAdjuster shooterAdjuster, Drivetrain drivetrain, Storage storage,
                 Shooter shooter, GetFunk getFunk) {
        this.getFunk = getFunk;
        addRequirements(shooterAdjuster, drivetrain, storage);
        addCommands(
                new RotateSwerveWithPID(drivetrain, drivetrain::rotation, this::getRequireRobotAngle,
                        drivetrain.pidSettings(), drivetrain.feedForwardSettings()),
                new SpeedUpShooter(shooterAdjuster.getName(), shooter.leftPidSettings(),
                        shooter.rightPidSettings(), shooter.leftFeedForwardSettings(), shooter.rightForwardSettings(),
                        this::getSpeed, this::getSpeed, shooter.leftMotor,shooter.rightMotor),
                new AdjustShooter("Shooty the Shooter the Shoot", shooter.leftPidSettings(),
                        shooter.rightPidSettings(), shooter.leftFeedForwardSettings(), shooter.rightForwardSettings(),
                        this::getSpeed, this::getSpeed, shooter::getLeftVelocity, shooter::getRightVelocity, shooter.leftMotor)
        );
    }

    public double getRequireShooterAngle(int x, int y) {
        if (x > 0.0 && x < 2.0) {
            //double angelFactor = getFunk.pose1;
        }
        return 0.0;
    }

    public double getRequireRobotAngle() {
        return 0.0;
    }

    public double getSpeed() {
        return 0.0;
    }
}
