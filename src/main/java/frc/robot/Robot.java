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
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    RootNamespace root = new RootNamespace("shooter sysid");
    Supplier<Double> voltage = root.addConstantDouble("ks calib", 0.0);
    Supplier<Double> setPoint = root.addConstantDouble("set point", 0.0);

    Ultrasonic thing = new Ultrasonic(new DigitalOutput(2), new DigitalInput(1));

    private Drivetrain drivetrain;

    //    Shooter shooter = new Shooter(new CANSparkMax(RobotMap.CAN.LEFT_SHOOTER_SPARK_MAX,
//            CANSparkLowLevel.MotorType.kBrushless),
//            new CANSparkMax(RobotMap.CAN.RIGHT_SHOOTER_SPARK_MAX , CANSparkLowLevel.MotorType.kBrushless));
    @Override
    public void robotInit() {
//        Supplier<Double> testSpeed = root.addConstantDouble("test speed", 0.0);
//        root.putRunnable("test adjuster back", () -> ShooterAdjuster.getInstance().getMotor().set(-1 * testSpeed.get()));
//        root.putRunnable("test adjuster front", () -> ShooterAdjuster.getInstance().getMotor().set(testSpeed.get()));
//        root.putRunnable("test adjuster stop", () -> ShooterAdjuster.getInstance().getMotor().set(0));
//        root.putRunnable("test shooter right back", () -> Shooter.getInstance().setRight(-1 * testSpeed.get()));
//        root.putRunnable("test shooter right front", () -> Shooter.getInstance().setRight(testSpeed.get()));
//        root.putRunnable("test shooter right stop", () -> Shooter.getInstance().setRight(0));
//        root.putRunnable("test shooter left back", () -> Shooter.getInstance().setLeft(-1 * testSpeed.get()));
//        root.putRunnable("test shooter left front", () -> Shooter.getInstance().setLeft(testSpeed.get()));
//        root.putRunnable("test shooter left stop", () -> Shooter.getInstance().setLeft(0));
//
        /*Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        sysid();
        */
//        DataLogManager.start("u/logs");
//        URCL.start();
//        sysid();
        drivetrain = Drivetrain.getInstance();
//        drivetrain.configureDashboard();
//        Shooter.getInstance().configureDashboard();
//        Shooter.getInstance().getLeftFlywheel().configureDashboard();
//        Shooter.getInstance().getRightFlywheel().configureDashboard();
//        ShooterAdjuster.getInstance().configureDashboard();
//        root.putCommand("shoot test", new Shoot(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(),
//                Storage.getInstance()));
        Ultrasonic.setAutomaticMode(true);
        root.putNumber("ultrasonic val", thing::getRangeMM);
        IntakePlacer.getInstance().configureDashboard();
        IntakeRoller.getInstance().configureDashboard();
//        Drivetrain.getInstance();4
//        Supplier<Double> setpoint = root.addConstantDouble("setpoint", 0);
//        root.putCommand("test both", new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getLeftFlywheel(),
//                Shooter.getInstance().getLeftFlywheel().pidSettings, Shooter.getInstance().getLeftFlywheel().feedForwardSettings, UnifiedControlMode.VELOCITY,
//                setpoint).alongWith(new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getRightFlywheel(),
//                Shooter.getInstance().getRightFlywheel().pidSettings, Shooter.getInstance().getRightFlywheel().feedForwardSettings, UnifiedControlMode.VELOCITY,
//                setpoint)));
//        DigitalInput limit = new DigitalInput(1);
//        root.putBoolean("limit val", limit::get);
//        root.putCommand("reset adjuster", ShooterAdjuster.getInstance().getResetCommand());
//        root.putCommand("shoot lmao", new Shoot(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(), Storage.getInstance()));
//        root.putCommand("intake?", new MoveGenericSubsystem(IntakeRoller.getInstance(), 1));
//        root.putCommand("close intake", new RunCommand(() -> IntakePlacer.getInstance().move()));
//        root.putCommand("intake note", new IntakeNote(IntakeRoller.getInstance(), Storage.getInstance(),
//                IntakePlacer.getInstance(), ShooterAdjuster.getInstance(), false).withTimeout(2.5));
//        root.putCommand("intake", new IntakeNote(IntakeRoller.getInstance(), Storage.getInstance(),
//                IntakePlacer.getInstance(), ShooterAdjuster.getInstance(), false));
        SwerveModuleHolder.getFrontLeft().configureDashboard();
        SwerveModuleHolder.getFrontRight().configureDashboard();
        SwerveModuleHolder.getBackLeft().configureDashboard();
        SwerveModuleHolder.getBackRight().configureDashboard();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        root.update();
        Shoot.ROOT.update();
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
//        new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getLeftFlywheel(), new PIDSettings(0, 0, 9999999),
//                FeedForwardSettings.EMPTY_FFSETTINGS, UnifiedControlMode.VOLTAGE, () -> 12.0).schedule();

//        Shooter.getInstance().setLeftVoltage(voltage.get());

//        new SpeedUpShooter(shooter, () -> 0.0, setPoint).schedule();
//        Shooter.getInstance().leftMotor.set(1);
//        SpeedUpShooter speedUpShooter = new SpeedUpShooter(Shooter.getInstance(), () -> 1000.0, () -> 1000.0);
//        speedUpShooter.schedule();
//        SwerveModule
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
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
