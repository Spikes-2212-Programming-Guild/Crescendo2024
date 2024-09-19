// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.auto.DriveStraight;
import frc.robot.commands.auto.JustShoot;
import frc.robot.commands.auto.YeetAndRetreat;
import frc.robot.commands.auto.YeetAndRetreatAmpSide;
import frc.robot.subsystems.*;
import frc.robot.util.LEDService;
import org.littletonrobotics.urcl.URCL;

import java.awt.*;

import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {

    private final RootNamespace root = new RootNamespace("robot");
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final SpikesLogger logger = new SpikesLogger("Robot");

    private Drivetrain drivetrain;
    private Shooter shooter;
    private ShooterAdjuster shooterAdjuster;
    private Storage storage;
    private ShooterFlywheel leftShooter;
    private ShooterFlywheel rightShooter;
    private IntakeRoller intakeRoller;
    private IntakePlacer intakePlacer;
    private LEDService led;

    @Override
    public void robotInit() {
//        DataLogManager.start();
//        URCL.start();

        drivetrain = Drivetrain.getInstance();
        shooter = Shooter.getInstance();
        shooterAdjuster = ShooterAdjuster.getInstance();
        storage = Storage.getInstance();
        leftShooter = ShooterFlywheel.getLeftInstance();
        rightShooter = ShooterFlywheel.getRightInstance();
        intakeRoller = IntakeRoller.getInstance();
        intakePlacer = IntakePlacer.getInstance();
        led = LEDService.getInstance();

        root.putCommand("shoot test", new Shoot(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(),
                Storage.getInstance(), Shoot.CLOSE_HEIGHT).getCommand());
        root.putCommand("intake note", new IntakeNote(IntakeRoller.getInstance(), Storage.getInstance(),
                IntakePlacer.getInstance(), ShooterAdjuster.getInstance(), false));
        NamedCommands.registerCommand("open-intake", new OpenIntake(intakePlacer));
        NamedCommands.registerCommand("close-intake", new CloseIntake(intakePlacer));
        NamedCommands.registerCommand("shoot-close", new Shoot(shooter, drivetrain, shooterAdjuster, storage,
                Shoot.CLOSE_HEIGHT).getCommand());
        NamedCommands.registerCommand("shoot-middle", new Shoot(shooter, drivetrain, shooterAdjuster, storage,
                Shoot.MIDDLE_HEIGHT).getCommand());
        NamedCommands.registerCommand("intake-note", new IntakeNote(intakeRoller, storage, intakePlacer, shooterAdjuster,
                false));
        NamedCommands.registerCommand("reset-adjuster", shooterAdjuster.getResetCommand());
        NamedCommands.registerCommand("reset-placer", new InstantCommand(intakePlacer::resetPosition));
        NamedCommands.registerCommand("reset-gyro", new InstantCommand(drivetrain::resetGyro));
        root.putCommand("2", new PathPlannerAuto("2"));

        autoChooser.addOption("single (middle)", new PathPlannerAuto("single middle"));
        autoChooser.addOption("double (middle)", new PathPlannerAuto("double middle"));
        autoChooser.addOption("single (short side)", new PathPlannerAuto("single short"));
        autoChooser.addOption("single (long side)", new PathPlannerAuto("single long side"));
        autoChooser.addOption("just shoot", new JustShoot(shooter, shooterAdjuster, intakePlacer, drivetrain, storage));
        root.putData("auto chooser", autoChooser);
    }

    @Override
    public void robotPeriodic() {
        logger.log(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? "red" : "blue");
        led.periodic();
        led.preGame();
        CommandScheduler.getInstance().run();
        root.update();
        Shoot.ROOT.update();
        ShootWithParameters.ROOT.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        Command autoCommand = autoChooser.getSelected() == null ?
                new JustShoot(shooter, shooterAdjuster, intakePlacer, drivetrain, storage) : autoChooser.getSelected();
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        OI oi = new OI();
        drivetrain.resetRelativeEncoders();
        drivetrain.setDefaultCommand(new DriveSwerve(drivetrain,
                () -> oi.getLeftY() * DriveSwerve.MAX_DRIVE_SPEED,
                () -> oi.getLeftX() * DriveSwerve.MAX_DRIVE_SPEED,
                () -> oi.getRightX() * DriveSwerve.MAX_TURN_SPEED,
                true, false));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
