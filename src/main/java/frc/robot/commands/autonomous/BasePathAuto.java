package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

public abstract class BasePathAuto {

    protected BasePathAuto(Drivetrain drivetrain) {
        Shooter shooter = Shooter.getInstance();
        ShooterAdjuster shooterAdjuster = ShooterAdjuster.getInstance();
        IntakePlacer intakePlacer = IntakePlacer.getInstance();
        IntakeRoller intakeRoller = IntakeRoller.getInstance();
        Storage storage = Storage.getInstance();
        NamedCommands.registerCommand("shoot", new Shoot(shooter, drivetrain, shooterAdjuster, storage));
        NamedCommands.registerCommand("open-intake", new OpenIntake(intakePlacer));
        NamedCommands.registerCommand("intake-note", new IntakeNote(intakeRoller, storage, intakePlacer, false));
    }

    public abstract Command getCommand();
}
