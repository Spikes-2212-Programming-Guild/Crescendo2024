package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.commands.IntakeNote;

public class IntakeRoller extends MotoredGenericSubsystem {

    private static final double MIN_SPEED = 0;
    private static final double MAX_SPEED = 0.8;

    public IntakeRoller(String name, CANSparkMax master) {
        super(name, MIN_SPEED, MAX_SPEED, master);
    }
}
