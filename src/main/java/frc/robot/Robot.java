// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

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

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        drivetrain.resetRelativeEncoders();
        DriveSwerve driveSwerve = new DriveSwerve(drivetrain,
                () -> 0.28,
                () -> 0.28,
                () -> 0.0,
                false, false);
        driveSwerve.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        drivetrain.resetRelativeEncoders();
        DriveSwerve driveSwerve = new DriveSwerve(drivetrain,
                () -> ps.getLeftX() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                () -> ps.getLeftY() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                () -> ps.getRightX() * Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                false, false);
        driveSwerve.schedule();
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
