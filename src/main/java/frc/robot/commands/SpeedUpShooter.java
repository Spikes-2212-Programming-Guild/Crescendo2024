package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class SpeedUpShooter extends ParallelCommandGroup {

    public SpeedUpShooter(Shooter shooter, Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        addCommands(
                new MoveSmartMotorControllerGenericSubsystem(shooter.getLeftFlywheel(), shooter.leftPIDSettings,
                        shooter.leftFeedForwardSettings, UnifiedControlMode.VELOCITY, leftSetpoint),
                new MoveSmartMotorControllerGenericSubsystem(shooter.getRightFlywheel(), shooter.rightPIDSettings,
                        shooter.rightFeedForwardSettings, UnifiedControlMode.VELOCITY, rightSetpoint));
    }
}
