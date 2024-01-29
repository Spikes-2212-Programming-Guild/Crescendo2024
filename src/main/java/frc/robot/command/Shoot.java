package frc.robot.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.Storage;

public class Shoot extends SequentialCommandGroup {

    public Shoot(ShooterAdjuster shooterAdjuster, Drivetrain drivetrain, Storage storage) {

    }

    public double getRequireShooterAngle()
    {
        return 0.0;
    }

    public double getRequireRobotAngle()
    {
        return 0.0;
    }

}
