package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.commands.IntakeNote;

public class IntakeRoller extends SparkGenericSubsystem {

    public IntakeRoller(String name, CANSparkMax master, CANSparkMax... slaves) {
        super(name, master, slaves);
    }
}
