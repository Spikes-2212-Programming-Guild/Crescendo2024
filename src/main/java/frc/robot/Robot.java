// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootWithParameters;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    RootNamespace root = new RootNamespace("shooter sysid");
    Supplier<Double> voltage = root.addConstantDouble("ks calib", 0.0);
    Supplier<Double> setPoint = root.addConstantDouble("set point", 0.0);

    private Drivetrain drivetrain;
    private Shooter shooter;
    private ShooterAdjuster shooterAdjuster;
    private Storage storage;
    private ShooterFlywheel leftShooter;
    private ShooterFlywheel rightShooter;
    private IntakeRoller intakeRoller;
    private IntakePlacer intakePlacer;

    @Override
    public void robotInit() {
        drivetrain = Drivetrain.getInstance();
        shooter = Shooter.getInstance();
        shooterAdjuster = ShooterAdjuster.getInstance();
        storage = Storage.getInstance();
        leftShooter = ShooterFlywheel.getLeftInstance();
        rightShooter = ShooterFlywheel.getRightInstance();
        intakeRoller = IntakeRoller.getInstance();

        root.putCommand("shoot test", new Shoot(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(),
                Storage.getInstance()));
        root.putCommand("intake note", new IntakeNote(IntakeRoller.getInstance(), Storage.getInstance(),
                IntakePlacer.getInstance(), ShooterAdjuster.getInstance(), false));
        root.putCommand("shoot2 test", new ShootWithParameters(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(),
                Storage.getInstance(), () -> 0.0, () -> 3000.0, () -> 400.0, () -> 24.8));
    }

    @Override
    public void robotPeriodic() {
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
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        OI oi = new OI();
//        ParallelRaceGroup openIntake = new RunCommand(intakePlacer::move).withTimeout(0.6);
//        openIntake.schedule();
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
        // Create the SysId routine
//        SysIDRoutine sysIdRoutine = new SysIDRoutine(
//                new SysIDRoutine.Config(),
//                new SysIDRoutine.Mechanism(
//                        voltageMeasure -> Shooter.getInstance().getRightFlywheel().setVoltage(voltageMeasure.in(Volts)),
//                        null, // No log consumer, since data is recorded by AdvantageKit
//                        Shooter.getInstance().getRightFlywheel()
//                )
//        );
//        root.putData("q forward", sysIdRoutine.quasistatic(SysIDRoutine.Direction.kForward));
//        root.putData("q backward", sysIdRoutine.quasistatic(SysIDRoutine.Direction.kReverse));
//        root.putData("d forward", sysIdRoutine.dynamic(SysIDRoutine.Direction.kForward));
//        root.putData("d backward", sysIdRoutine.dynamic(SysIDRoutine.Direction.kReverse));
    }
}
