package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class IntakeFromFeeder extends ParallelCommandGroup {

    private static final double SHOOTER_SPEED = -0.5;

    public IntakeFromFeeder(Shooter shooter, Storage storage) {
        addCommands(
                new MoveSmartMotorControllerGenericSubsystem(shooter.getLeftFlywheel(), new PIDSettings(0, 0, 1000),
                        new FeedForwardSettings(0, 0), UnifiedControlMode.VELOCITY, () -> SHOOTER_SPEED)
                        .until(storage::seesNote)
        );
    }
}
