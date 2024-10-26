package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class OI /*GEVALD*/ {

    private static final double STORAGE_RETRACTION_SPEED = 0.2;
    private static final Supplier<Double> SHOOTER_RETRACTION_SPEED_ROTATIONS_PER_MINUTE = () -> -2000.0;

    private final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    private final XboxControllerWrapper xbox = new XboxControllerWrapper(1);

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Storage storage = Storage.getInstance();
    private final ShooterAdjuster shooterAdjuster = ShooterAdjuster.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final IntakeRoller intakeRoller = IntakeRoller.getInstance();
    private final IntakePlacer intakePlacer = IntakePlacer.getInstance();

    public OI() {
        // Resets gyro - R1
        ps.getR1Button().onTrue(new InstantCommand(drivetrain::resetGyro));
        // Intakes a note - A
        xbox.getGreenButton().onTrue(new IntakeNote(intakeRoller, storage, intakePlacer, shooterAdjuster, false));
        // Shoots the note from the subwoofer - B
        xbox.getRedButton().onTrue(new Shoot(shooter, drivetrain, shooterAdjuster, storage,
                Shoot.SUBWOOFER_ADJUSTER_HEIGHT).getCommand());
        // Shoots the note from the safe zone - X
        xbox.getBlueButton().onTrue(new Shoot(shooter, drivetrain, shooterAdjuster, storage,
                Shoot.SAFE_ZONE_ADJUSTER_HEIGHT).getCommand());
        // Moves the storage backwards - LT
        xbox.getLTButton().whileTrue(new MoveGenericSubsystem(storage, STORAGE_RETRACTION_SPEED));
        // Moves the shooter backwards - RT
        xbox.getRTButton().whileTrue(new SpeedUpShooter(shooter, SHOOTER_RETRACTION_SPEED_ROTATIONS_PER_MINUTE,
                SHOOTER_RETRACTION_SPEED_ROTATIONS_PER_MINUTE));
        // Closes the intake - UP
        xbox.getUpButton().onTrue(new CloseIntake(intakePlacer));
        // Opens the intake - DOWN
        xbox.getDownButton().onTrue(new OpenIntake(intakePlacer));
        // Stops all subsystems - LEFT
        xbox.getLeftButton().onTrue(new InstantCommand(() -> {
            storage.stop();
            intakeRoller.stop();
            shooterAdjuster.stop();
            shooter.stop();
        }, storage, intakeRoller, shooterAdjuster, shooter));
    }

    public double getLeftX() {
        return -ps.getLeftX();
    }

    public double getLeftY() {
        return -ps.getLeftY();
    }

    public double getRightX() {
        return ps.getRightX();
    }
}
