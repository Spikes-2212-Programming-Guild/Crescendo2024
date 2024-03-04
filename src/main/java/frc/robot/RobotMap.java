package frc.robot;

public class RobotMap {

    public interface CAN {

        int FRONT_LEFT_DRIVE_SPARK_MAX = 1;
        int FRONT_LEFT_TURN_SPARK_MAX = 2;
        int FRONT_LEFT_ABSOLUTE_ENCODER = 3;

        int FRONT_RIGHT_DRIVE_SPARK_MAX = 4;
        int FRONT_RIGHT_TURN_SPARK_MAX = 5;
        int FRONT_RIGHT_ABSOLUTE_ENCODER = 6;

        int BACK_LEFT_DRIVE_SPARK_MAX = 7;
        int BACK_LEFT_TURN_SPARK_MAX = 8;
        int BACK_LEFT_ABSOLUTE_ENCODER = 9;

        int BACK_RIGHT_DRIVE_SPARK_MAX = 10;
        int BACK_RIGHT_TURN_SPARK_MAX = 11;
        int BACK_RIGHT_ABSOLUTE_ENCODER = 12;

        int LEFT_SHOOTER_SPARK_MAX = 13;
        int RIGHT_SHOOTER_SPARK_MAX = 14;

        int SHOOTER_ADJUSTER_SPARK_MAX = 15;

        int INTAKE_ROLLER_SPARK_MAX = 19;

        int INTAKE_PLACER_LEFT_SPARK_MAX = 17;
        int INTAKE_PLACER_RIGHT_SPARK_MAX = 18;

        int STORAGE_SPARK_MAX = 16;
    }
    
    public interface DIO {

        int INTAKE_PLACER_TOP_LIMIT_SWITCH = 5;
        int INTAKE_PLACER_BOTTOM_LIMIT_SWITCH = 6;

        int SHOOTER_ADJUSTER_ABSOLUTE_ENCODER = 0;
        int SHOOTER_ADJUSTER_TOP_HALL_EFFECT = 7;
        int SHOOTER_ADJUSTER_BOTTOM_LIMIT = 8;

        int STORAGE_ULTRASONIC_OUTPUT = 2;
        int STORAGE_ULTRASONIC_INPUT = 1;
        int STORAGE_IR = 4;
    }
    
    public interface PWM {
        int LED = 7;
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
