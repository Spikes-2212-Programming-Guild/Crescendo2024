package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

public class OI /*GEVALD*/{

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
        // Stops Intake - Left
        xbox.getLeftButton().onTrue(new InstantCommand(() -> {}, intakeRoller, storage, intakeRoller, shooterAdjuster));
        // Shoots the note - B
        xbox.getRedButton().onTrue(new Shoot(shooter, drivetrain, shooterAdjuster, storage));
        // Moves the storage backwards - Y
        xbox.getYellowButton().whileTrue(new MoveGenericSubsystem(storage, 0.2));
        // Resets the adjuster - X
        xbox.getBlueButton().onTrue(shooterAdjuster.getResetCommand());
        // Moves the adjuster up - RB
        xbox.getRBButton().whileTrue(new InstantCommand(shooterAdjuster::move)).onFalse(
                new InstantCommand(shooterAdjuster::stop));
        // Moves the adjuster down - LB
        xbox.getLBButton().whileTrue(new InstantCommand(shooterAdjuster::unmove)).onFalse(
                new InstantCommand(shooterAdjuster::stop));
    }

    public double getLeftX() {
        return ps.getLeftX();
    }

    public double getLeftY() {
        return ps.getLeftY();
    }

    public double getRightX() {
        return ps.getRightX();
    }
}
