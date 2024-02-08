// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    CANSparkMax left;
    CANSparkMax right;
    CANSparkMax adjuster;
    CANSparkMax storage;

    @Override
    public void robotInit() {
        left = new CANSparkMax(RobotMap.CAN.LEFT_SHOOTER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless);
        right = new CANSparkMax(RobotMap.CAN.RIGHT_SHOOTER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless);
        adjuster = new CANSparkMax(RobotMap.CAN.SHOOTER_ADJUSTER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless);
        storage = new CANSparkMax(RobotMap.CAN.STORAGE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless);
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
        left.set(1);
        right.set(1);

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
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
