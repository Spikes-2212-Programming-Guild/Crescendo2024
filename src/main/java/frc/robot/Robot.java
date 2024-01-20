// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainImpl;

public class Robot extends TimedRobot {

    private final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    private DrivetrainImpl drivetrain;

    @Override
    public void robotInit() {
        drivetrain = DrivetrainImpl.getInstance();
        drivetrain.resetRelativeEncoders();
        drivetrain.configureDashboard();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        drivetrain.periodic();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        DriveSwerve driveSwerve = new DriveSwerve(drivetrain, () -> 2.0, () -> 0.0, () -> 0.0, false, false);
        driveSwerve.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        DriveSwerve driveSwerve = new DriveSwerve(drivetrain,
                () -> ps.getLeftY() * DrivetrainImpl.MAX_SPEED_METERS_PER_SECONDS,
                () -> ps.getLeftX() * DrivetrainImpl.MAX_SPEED_METERS_PER_SECONDS,
                () -> ps.getRightX() * DrivetrainImpl.MAX_SPEED_METERS_PER_SECONDS  ,
                true, false);
        drivetrain.setDefaultCommand(driveSwerve);
        ps.getR1Button().onTrue(new InstantCommand(() -> drivetrain.resetGyro()));
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
