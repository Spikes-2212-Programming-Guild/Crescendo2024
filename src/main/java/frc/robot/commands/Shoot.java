package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

public class Shoot extends ParallelCommandGroup {

    public Shoot(ShooterAdjuster shooterAdjuster, Drivetrain drivetrain, Storage storage, Shooter shooter) {
        addRequirements(shooterAdjuster, drivetrain, storage);
        addCommands(
                new RotateSwerveWithPID(drivetrain, drivetrain::rotation, this::getRequireRobotAngle,
                        drivetrain.pidSettings(), drivetrain.feedForwardSettings()),
                new MoveSmartMotorControllerSubsystemTrapezically(shooterAdjuster, shooterAdjuster.pidSettings(),
                        shooterAdjuster.feedForwardSettings(), this::getRequireShooterAngle,
                        shooterAdjuster.trapezoidProfileSettings()),
                new SequentialCommandGroup(new MoveShooter("Shooty the Shooter the Shoot", shooter.leftPidSettings() ,
                        shooter.rightPidSettings(), shooter.leftFeedForwardSettings(),shooter.rightForwardSettings(),
                        this::getSpeed,this::getSpeed,shooter::getLeftVelocity,shooter::getRightVelocity, shooter.master
                        ,Shooter.slave),
                        new );
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
