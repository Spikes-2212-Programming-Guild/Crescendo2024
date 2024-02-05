// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private RootNamespace root = new RootNamespace("root");

    @Override
    public void robotInit() {
        Drivetrain.getInstance().resetRelativeEncoders();
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
        new DriveSwerve(Drivetrain.getInstance(), () -> 2.0, () -> 0.0, () -> 0.0, true, false).schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

        PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

        Supplier<Double> leftX = ps::getLeftX;
        Supplier<Double> leftY = ps::getLeftY;
        Supplier<Double> rightX = ps::getRightX;

        root.putNumber("left x", leftX);
        root.putNumber("left y", leftY);
        root.putNumber("rotation x", rightX);

        ps.getR1Button().onTrue(new InstantCommand(Drivetrain.getInstance()::resetGyro));


        Drivetrain.getInstance().setDefaultCommand(new DriveSwerve(Drivetrain.getInstance(),
                () -> -leftY.get() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                () -> -leftX.get() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                () -> rightX.get() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
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
