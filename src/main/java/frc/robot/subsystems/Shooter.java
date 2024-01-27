package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

public abstract class Shooter{
    public final CANSparkBase leftMotor;
    public final CANSparkBase rightMotor;

    public PIDSettings leftPidSettings(){
        return null;
    }
    public PIDSettings rightPidSettings(){
        return null;
    }
    public FeedForwardSettings leftFeedForwardSettings(){
        return null;
    }
    public FeedForwardSettings rightForwardSettings(){
        return null;
    }
    public double getLeftVelocity(){
        return 0.0;
    }
    public double getRightVelocity(){
        return 0.0;
    }

    public Shooter(CANSparkBase leftMotor, CANSparkBase rightMotor) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

}
