package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.*;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

/**
 * Command which is responsible for speeding up each side of the shooter.
 */
public class SpeedUpShooter extends ParallelCommandGroup {

    public SpeedUpShooter(Shooter shooter, Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        addCommands(
                new MoveSmartMotorControllerGenericSubsystem(shooter.getLeftFlywheel(),
                        shooter.getLeftFlywheel().getPIDSettings(), shooter.getLeftFlywheel().getFeedForwardSettings(),
                        UnifiedControlMode.VELOCITY, leftSetpoint) {
                    @Override
                    public void end(boolean interrupted) {
                    }
                },
                new MoveSmartMotorControllerGenericSubsystem(shooter.getRightFlywheel(),
                        shooter.getRightFlywheel().getPIDSettings(), shooter.getRightFlywheel().getFeedForwardSettings(),
                        UnifiedControlMode.VELOCITY, rightSetpoint) {
                    @Override
                    public void end(boolean interrupted) {
                    }
                });
    }
}
