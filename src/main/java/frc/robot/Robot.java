// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpeedUpShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {

    RootNamespace root = new RootNamespace("shooter sysid");
    Supplier<Double> voltage = root.addConstantDouble("ks calib", 0.0);
    Supplier<Double> setPoint = root.addConstantDouble("set point", 0.0);

    DigitalInput limit = new DigitalInput(1);

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
        Shooter.getInstance().configureDashboard();
        Shooter.getInstance().getLeftFlywheel().configureDashboard();
        Shooter.getInstance().getRightFlywheel().configureDashboard();
        ShooterAdjuster.getInstance().configureDashboard();
//        Drivetrain.getInstance();4
        Supplier<Double> setpoint = root.addConstantDouble("setpoint", 0);
        root.putCommand("test left", new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getLeftFlywheel(),
                Shooter.getInstance().getLeftFlywheel().pidSettings, Shooter.getInstance().getLeftFlywheel().feedForwardSettings, UnifiedControlMode.VELOCITY,
                setpoint));
        root.putCommand("test right", new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getRightFlywheel(),
                Shooter.getInstance().getRightFlywheel().pidSettings, Shooter.getInstance().getRightFlywheel().feedForwardSettings, UnifiedControlMode.VELOCITY,
                setpoint));
        root.putBoolean("limit val", limit::get);
        root.putCommand("reset adjuster", ShooterAdjuster.getInstance().getResetCommand());
        root.putCommand("shoot lmao", new Shoot(Shooter.getInstance(), Drivetrain.getInstance(), ShooterAdjuster.getInstance(), Storage.getInstance()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        root.update();
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
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        root.putCommand("start left", new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getLeftFlywheel(),
                new PIDSettings(0, 0, 1000), new FeedForwardSettings(0, 0, 0), UnifiedControlMode.VOLTAGE, () -> 0.4872));
        root.putCommand("start right", new MoveSmartMotorControllerGenericSubsystem(Shooter.getInstance().getRightFlywheel(),
                new PIDSettings(0, 0, 1000), new FeedForwardSettings(0, 0, 0), UnifiedControlMode.VOLTAGE, () -> 12.0));
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
        SysIDRoutine sysIdRoutine = new SysIDRoutine(
                new SysIDRoutine.Config(),
                new SysIDRoutine.Mechanism(
                        voltageMeasure -> Shooter.getInstance().getRightFlywheel().setVoltage(voltageMeasure.in(Volts)),
                        null, // No log consumer, since data is recorded by AdvantageKit
                        Shooter.getInstance().getRightFlywheel()
                )
        );
        root.putData("q forward", sysIdRoutine.quasistatic(SysIDRoutine.Direction.kForward));
        root.putData("q backward", sysIdRoutine.quasistatic(SysIDRoutine.Direction.kReverse));
        root.putData("d forward", sysIdRoutine.dynamic(SysIDRoutine.Direction.kForward));
        root.putData("d backward", sysIdRoutine.dynamic(SysIDRoutine.Direction.kReverse));
    }
}
