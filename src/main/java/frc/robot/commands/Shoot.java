package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class Shoot extends ParallelCommandGroup {

    public Shoot(frc.robot.subsystems.ShooterAdjuster shooterAdjuster, Drivetrain drivetrain, Storage storage, Shooter shooter) {
        addRequirements(shooterAdjuster, drivetrain, storage);
        addCommands(
                new RotateSwerveWithPID(drivetrain, drivetrain::rotation, this::getRequireRobotAngle,
                        drivetrain.pidSettings(), drivetrain.feedForwardSettings()),
               // new AdjustShooter(shooterAdjuster.getName(), shooterAdjuster.pidSettings(),
                 //       shooterAdjuster.feedForwardSettings(), this::getRequireShooterAngle,
                   //     shooterAdjuster.trapezoidProfileSettings()),
                new AdjustShooter("Shooty the Shooter the Shoot", shooter.leftPidSettings() ,
                        shooter.rightPidSettings(), shooter.leftFeedForwardSettings(),shooter.rightForwardSettings(),
                        this::getSpeed,this::getSpeed,shooter::getLeftVelocity,shooter::getRightVelocity, shooter.master
                        ,Shooter.slave)
        );
    }
    public double getRequireShooterAngle() {
        return 0.0;
    }
    public double getRequireRobotAngle() {
        return 0.0;
    }
    public double getSpeed(){
        return 0.0;
    }
}
