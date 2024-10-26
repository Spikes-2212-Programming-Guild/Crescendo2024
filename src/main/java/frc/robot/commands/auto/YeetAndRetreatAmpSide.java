package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

/**
 * Auto which starts on the amp side, shoots the starting note and drives out of the wing.
 */
public class YeetAndRetreatAmpSide extends SequentialCommandGroup {

    private static final double MOVE_TIMEOUT = 4;
    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;
    private static final Supplier<Double> STRAFE_SPEED = YeetAndRetreatAmpSide::getStrafeSign;

    //Click A-Stop if necessary!
    public YeetAndRetreatAmpSide(Drivetrain drivetrain, Shooter shooter, ShooterAdjuster adjuster, Storage storage,
                                 IntakePlacer intakePlacer) {
        addCommands(
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new Shoot(shooter, drivetrain, adjuster, storage, Shoot.SUBWOOFER_ADJUSTER_HEIGHT).getCommand(),
                new DriveSwerve(drivetrain, DRIVE_SPEED, STRAFE_SPEED, () -> 0.0, false,
                        false).withTimeout(MOVE_TIMEOUT)
        );
    }

    // depending on the alliance, the strafe direction changes as the amps are on opposite sides
    private static double getStrafeSign() {
        if (DriverStation.getAlliance().isEmpty()) return 0;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return 1;
        return -1;
    }
}
