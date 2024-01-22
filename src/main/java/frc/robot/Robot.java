// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.RotateWithPID;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    private static final Supplier<Double> angle = namespace.addConstantDouble("angle", 0);
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
        drivetrain.resetRelativeEncoders();
        RotateWithPID rotateWithPID = new RotateWithPID(drivetrain, () -> 100.0, () -> (drivetrain.getAngle() % 360),
                new PIDSettings(0.05, 0, 0.0005, 0.2, 0.2), new FeedForwardSettings(0.015, 0.21277, 0));
        rotateWithPID.schedule();
//        RunCommand command = new RunCommand(drivetrain::iAmDeathDestroyerOfWorlds) {
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        };
//        command.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        drivetrain.resetRelativeEncoders();
//        drivetrain.resetGyro();
        DriveSwerve driveSwerve = new DriveSwerve(drivetrain,
                () -> ps.getLeftY() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                () -> ps.getLeftX() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                () -> ps.getRightX() * 10,
                true, false);
        ps.getR1Button().onTrue(new InstantCommand(drivetrain::resetGyro));
        drivetrain.setDefaultCommand(driveSwerve);
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
