package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class YeetAndRetreatAmpSide extends SequentialCommandGroup {

    private static final double MOVE_TIMEOUT = 4;
    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;
    private static final Supplier<Double> SIDE_DRIVE_SPEED = () -> getSideDriveSign() * 1.0;

    //Click A-Stop if necessary!
    public YeetAndRetreatAmpSide(Drivetrain drivetrain, Shooter shooter, ShooterAdjuster adjuster, Storage storage, IntakePlacer intakePlacer) {
        addCommands(
                new InstantCommand(intakePlacer::resetPosition),
                adjuster.getResetCommand(),
                new Shoot(shooter, drivetrain, adjuster, storage, Shoot.CLOSE_HEIGHT).getCommand(),
                new DriveSwerve(drivetrain, DRIVE_SPEED, SIDE_DRIVE_SPEED, () -> 0.0, false, false).withTimeout(MOVE_TIMEOUT)
        );
    }

    private static int getSideDriveSign() {
        if (DriverStation.getAlliance().isEmpty()) return 0;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return 1;
        return -1;
    }
}
