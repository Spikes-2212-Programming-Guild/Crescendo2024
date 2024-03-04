// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootWithParameters;
import frc.robot.commands.auto.DriveStraight;
import frc.robot.commands.auto.JustShoot;
import frc.robot.commands.auto.YeetAndRetreat;
import frc.robot.commands.auto.YeetAndRetreatAmpSide;
import frc.robot.subsystems.*;
import frc.robot.util.LEDService;

import java.awt.*;

import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {

    int foodAnimation = 0;
    int snakeAnimation = 0;
    int explosionAnimation = 0;
    boolean invert;
    boolean explosion;

    private final RootNamespace root = new RootNamespace("robot");
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private Drivetrain drivetrain;
    private Shooter shooter;
    private ShooterAdjuster shooterAdjuster;
    private Storage storage;
    private ShooterFlywheel leftShooter;
    private ShooterFlywheel rightShooter;
    //    private IntakeRoller intakeRoller;
    private IntakePlacer intakePlacer;
    LEDService led = LEDService.getInstance();

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
//        intakeRoller = IntakeRoller.getInstance();
        intakePlacer = IntakePlacer.getInstance();

        root.putCommand("shoot test", new Shoot(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(),
                Storage.getInstance(), Shoot.CLOSE_HEIGHT));
        root.putCommand("intake note", new IntakeNote(IntakeRoller.getInstance(), Storage.getInstance(),
                IntakePlacer.getInstance(), ShooterAdjuster.getInstance(), false));

        autoChooser.addOption("single (middle and long side)", new YeetAndRetreat(drivetrain, shooter, shooterAdjuster, storage, intakePlacer));
//        autoChooser.addOption("double (middle)",
//                new YeetAndRetreatAndYeet(drivetrain, shooter, shooterAdjuster, intakePlacer, intakeRoller, storage));
        autoChooser.addOption("single (short side)", new YeetAndRetreatAmpSide(drivetrain, shooter, shooterAdjuster, storage, intakePlacer));
        autoChooser.addOption("drive straight", new DriveStraight(drivetrain, shooterAdjuster, intakePlacer));
        autoChooser.addOption("just shoot", new JustShoot(shooter, shooterAdjuster, intakePlacer, drivetrain, storage));
        root.putData("auto chooser", autoChooser);
        root.putData("test", new InstantCommand(() -> leftShooter.setVoltage(12)));
//        sysid();
    }

    @Override
    public void robotPeriodic() {
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
                new DriveStraight(drivetrain, shooterAdjuster, intakePlacer) : autoChooser.getSelected();
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

    private void sysid() {
//         Create the SysId routine
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> {
                            drivetrain.setAnglesToZero();
                            drivetrain.setVoltage(voltageMeasure.in(Volts));
                        },
                        null,
                        drivetrain
                )
        );
        root.putData("q forward", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        root.putData("q backward", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        root.putData("d forward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        root.putData("d backward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}
