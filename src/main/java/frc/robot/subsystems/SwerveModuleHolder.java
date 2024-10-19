package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.RobotMap;

// https://www.youtube.com/watch?v=jxcMRkqaQdw

/**
 * This class contains all 4 swerve modules.
 */
public class SwerveModuleHolder {

    private static final RootNamespace namespace = new RootNamespace("swerve module holder");

    private static final String FRONT_LEFT_NAMESPACE_NAME = "front left";
    private static final String FRONT_RIGHT_NAMESPACE_NAME = "front right";
    private static final String BACK_LEFT_NAMESPACE_NAME = "back left";
    private static final String BACK_RIGHT_NAMESPACE_NAME = "back right";

    private static final boolean FRONT_LEFT_DRIVE_INVERTED = true;
    private static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
    private static final boolean BACK_LEFT_DRIVE_INVERTED = true;
    private static final boolean BACK_RIGHT_DRIVE_INVERTED = true;

    private static final boolean FRONT_LEFT_CANCODER_INVERTED = false;
    private static final boolean FRONT_RIGHT_CANCODER_INVERTED = false;
    private static final boolean BACK_LEFT_CANCODER_INVERTED = false;
    private static final boolean BACK_RIGHT_CANCODER_INVERTED = false;

    //all in rotations
    private static final double FRONT_LEFT_OFFSET = -0.821289;
    private static final double FRONT_RIGHT_OFFSET = -0.210938;
    private static final double BACK_LEFT_OFFSET = -0.565918;
    private static final double BACK_RIGHT_OFFSET = -0.157715;

    private static final PIDSettings drivePIDSettings = namespace.addPIDNamespace("drive",
            new PIDSettings(-1, -1, -1, -1, -1));
    private static final PIDSettings turnPIDSettings = namespace.addPIDNamespace("turn",
            new PIDSettings(0.004, 0, 0, 0, 0));
    private static final FeedForwardSettings driveFeedForwardSettings = namespace.addFeedForwardNamespace("drive",
            new FeedForwardSettings(-1, -1, -1, -1));
    private static final FeedForwardSettings turnFeedForwardSettings = namespace.addFeedForwardNamespace("turn",
            new FeedForwardSettings(0.29, 0, 0, 0));

    private static SwerveModule frontLeft;
    private static SwerveModule frontRight;
    private static SwerveModule backLeft;
    private static SwerveModule backRight;

    public static SwerveModule getFrontLeft() {
        if (frontLeft == null) {
            frontLeft = new SwerveModule(FRONT_LEFT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.FRONT_LEFT_DRIVE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.FRONT_LEFT_TURN_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.FRONT_LEFT_ABSOLUTE_ENCODER), FRONT_LEFT_CANCODER_INVERTED,
                    drivePIDSettings, driveFeedForwardSettings, turnPIDSettings, turnFeedForwardSettings,
                    FRONT_LEFT_DRIVE_INVERTED, FRONT_LEFT_OFFSET);
        }
        return frontLeft;
    }

    public static SwerveModule getFrontRight() {
        if (frontRight == null) {
            frontRight = new SwerveModule(FRONT_RIGHT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.FRONT_RIGHT_DRIVE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.FRONT_RIGHT_TURN_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.FRONT_RIGHT_ABSOLUTE_ENCODER), FRONT_RIGHT_CANCODER_INVERTED,
                    drivePIDSettings, driveFeedForwardSettings, turnPIDSettings, turnFeedForwardSettings,
                    FRONT_RIGHT_DRIVE_INVERTED, FRONT_RIGHT_OFFSET);
        }
        return frontRight;
    }

    public static SwerveModule getBackLeft() {
        if (backLeft == null) {
            backLeft = new SwerveModule(BACK_LEFT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.BACK_LEFT_DRIVE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.BACK_LEFT_TURN_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.BACK_LEFT_ABSOLUTE_ENCODER), BACK_LEFT_CANCODER_INVERTED,
                    drivePIDSettings, driveFeedForwardSettings, turnPIDSettings, turnFeedForwardSettings,
                    BACK_LEFT_DRIVE_INVERTED, BACK_LEFT_OFFSET);
        }
        return backLeft;
    }

    public static SwerveModule getBackRight() {
        if (backRight == null) {
            backRight = new SwerveModule(BACK_RIGHT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.BACK_RIGHT_DRIVE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.BACK_RIGHT_TURN_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.BACK_RIGHT_ABSOLUTE_ENCODER), BACK_RIGHT_CANCODER_INVERTED,
                    drivePIDSettings, driveFeedForwardSettings, turnPIDSettings, turnFeedForwardSettings,
                    BACK_RIGHT_DRIVE_INVERTED, BACK_RIGHT_OFFSET);
        }
        return backRight;
    }
}
