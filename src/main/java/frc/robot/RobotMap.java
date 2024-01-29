package frc.robot;

public class RobotMap {

    public interface CAN {

        int SHOOTER_ADJUSTER_SPARKMAX = -1;
        int STORAGE_SPARK_MAX = -1;
    }
    
    public interface DIO {

        int SHOOTER_ADJUSTER_ABSOLUTE_ENCODER = -1;
        int SHOOTER_ADJUSTER_TOP_LIMIT = -1;
        int SHOOTER_ADJUSTER_BOTTOM_LIMIT = -1;
        int STORAGE_LIMIT = -1;
    }
    
    public interface PWM {

    }
    
    public interface AIN {
    
    }

    public interface PCM {

    }

    public interface RPH {

        int LEFT_CLIMBER_FORWARD = 0;
        int LEFT_CLIMBER_BACKWARD = 1;
        int RIGHT_CLIMBER_FORWARD = 2;
        int RIGHT_CLIMBER_BACKWARD = 3;

    }
}
