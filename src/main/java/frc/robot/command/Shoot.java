package frc.robot.command;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

public class Shoot extends SequentialCommandGroup {

    public Shoot(ShooterAdjuster shooterAdjuster, Drivetrain drivetrain, Storage storage) {
        addRequirements(shooterAdjuster, drivetrain, storage);
        addCommands(
                new RotateSwerveWithPID(drivetrain, drivetrain::rotation, this::getRequireRobotAngle,
                        drivetrain.pidSettings(), drivetrain.feedForwardSettings()),
                new MoveSmartMotorControllerSubsystemTrapezically(shooterAdjuster, shooterAdjuster.pidSettings(),
                        shooterAdjuster.feedForwardSettings(), this::getRequireShooterAngle,
                        shooterAdjuster.trapezoidProfileSettings())
        );
    }

    public double getRequireShooterAngle() {
        return 0.0;
    }

    public double getRequireRobotAngle() {
        return 0.0;
    }

}
