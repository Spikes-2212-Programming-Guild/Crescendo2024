package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class SpeedUpShooter extends ParallelCommandGroup {

    public SpeedUpShooter(Shooter shooter, Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        addCommands(
                new MoveSmartMotorControllerGenericSubsystem(shooter.getLeftFlywheel(), shooter.getLeftFlywheel().pidSettings,
                        shooter.getLeftFlywheel().feedForwardSettings, UnifiedControlMode.VELOCITY, leftSetpoint) {
                    @Override
                    public void end(boolean interrupted) {
                    }
                },
                new MoveSmartMotorControllerGenericSubsystem(shooter.getRightFlywheel(), shooter.getRightFlywheel().pidSettings,
                        shooter.getRightFlywheel().feedForwardSettings, UnifiedControlMode.VELOCITY, rightSetpoint) {
                    @Override
                    public void end(boolean interrupted) {
                    }
                });
    }
}
