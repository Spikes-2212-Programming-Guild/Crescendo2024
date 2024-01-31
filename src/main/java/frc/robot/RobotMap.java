package frc.robot;

public class RobotMap {

    public interface CAN {

        int FRONT_LEFT_DRIVE_SPARKMAX = 1;
        int FRONT_LEFT_TURN_SPARKMAX = 2;
        int FRONT_LEFT_ABSOLUTE_ENCODER = 3;

        int FRONT_RIGHT_DRIVE_SPARKMAX = 4;
        int FRONT_RIGHT_TURN_SPARKMAX = 5;
        int FRONT_RIGHT_ABSOLUTE_ENCODER = 6;

        int BACK_LEFT_DRIVE_SPARKMAX = 7;
        int BACK_LEFT_TURN_SPARKMAX = 8;
        int BACK_LEFT_ABSOLUTE_ENCODER = 9;

        int BACK_RIGHT_DRIVE_SPARKMAX = 10;
        int BACK_RIGHT_TURN_SPARKMAX = 11;
        int BACK_RIGHT_ABSOLUTE_ENCODER = 12;
        int INTAKE_ROLLER_SPARKMAX = -1;
        int INTAKE_PLACER_SPARKMAX = -1;
        int SHOOTER_ADJUSTER_SPARKMAX = -1;
        int STORAGE_SPARK_MAX = -1;
    }
    
    public interface DIO {

        int INTAKE_PLACER_TOP_LIMIT_SWITCH = -1;
        int INTAKE_PLACER_BOTTOM_LIMIT_SWITCH = -1;
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
