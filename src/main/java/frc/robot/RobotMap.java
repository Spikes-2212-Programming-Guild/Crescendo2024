package frc.robot;

public class RobotMap {

    public interface CAN {

        int SHOOTER_ADJUSTER_LEFT_SPARKMAX = Integer.MAX_VALUE;
        int SHOOTER_ADJUSTER_RIGHT_SPARKMAX = Integer.MAX_VALUE;
    }
    
    public interface DIO {

        int SHOOTER_ADJUSTER_ABSOLUTE_ENCODER = Integer.MIN_VALUE;
        int SHOOTER_ADJUSTER_TOP_LIMIT = Integer.MIN_VALUE;
        int SHOOTER_ADJUSTER_BOTTOM_LIMIT = Integer.MIN_VALUE;
    }
    
    public interface PWM {

    }
    
    public interface AIN {
    
    }

    public interface PCM {

    }
}
