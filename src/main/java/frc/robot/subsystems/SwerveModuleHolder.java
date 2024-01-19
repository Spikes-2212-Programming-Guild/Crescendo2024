package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.RobotMap;

import java.util.function.Supplier;

// https://www.youtube.com/watch?v=jxcMRkqaQdw
public class SwerveModuleHolder {

    private static final RootNamespace namespace = new RootNamespace("swerve module holder");

    private static final String FRONT_LEFT_NAMESPACE_NAME = "front left";
    private static final String FRONT_RIGHT_NAMESPACE_NAME = "front right";
    private static final String BACK_LEFT_NAMESPACE_NAME = "back left";
    private static final String BACK_RIGHT_NAMESPACE_NAME = "back right";

    private static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    private static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    private static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    private static final boolean BACK_RIGHT_DRIVE_INVERTED = false;

    private static final boolean FRONT_LEFT_CANCODER_INVERTED = false;
    private static final boolean FRONT_RIGHT_CANCODER_INVERTED = false;
    private static final boolean BACK_LEFT_CANCODER_INVERTED = false;
    private static final boolean BACK_RIGHT_CANCODER_INVERTED = false;

    private static final double FRONT_LEFT_OFFSET = 0;
    private static final double FRONT_RIGHT_OFFSET = 0;
    private static final double BACK_LEFT_OFFSET = 0;
    private static final double BACK_RIGHT_OFFSET = 0;

    private static final PIDSettings drivePIDSettings = namespace.addPIDNamespace("drive",
            new PIDSettings(0, 0, 0, 0, 0));
    private static final PIDSettings turnPIDSettings = namespace.addPIDNamespace("drive",
            new PIDSettings(0, 0, 0, 0, 0));
    private static final FeedForwardSettings driveFeedForwardSettings = namespace.addFeedForwardNamespace("drive",
            new FeedForwardSettings(0, 0, 0, 0));

    private static SwerveModule frontLeft;
    private static SwerveModule frontRight;
    private static SwerveModule backLeft;
    private static SwerveModule backRight;

    public static SwerveModule getFrontLeft() {
        if (frontLeft == null) {
            frontLeft = new SwerveModule(FRONT_LEFT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.FRONT_LEFT_DRIVE_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.FRONT_LEFT_TURN_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.FRONT_LEFT_ABSOLUTE_ENCODER), FRONT_LEFT_DRIVE_INVERTED,
                    FRONT_LEFT_CANCODER_INVERTED, FRONT_LEFT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings);
        }
        return frontLeft;
    }

    public static SwerveModule getFrontRight() {
        if (frontRight == null) {
            frontRight = new SwerveModule(FRONT_RIGHT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.FRONT_RIGHT_DRIVE_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.FRONT_RIGHT_TURN_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.FRONT_RIGHT_ABSOLUTE_ENCODER), FRONT_RIGHT_DRIVE_INVERTED,
                    FRONT_RIGHT_CANCODER_INVERTED, FRONT_RIGHT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings);
        }
        return frontRight;
    }

    public static SwerveModule getBackLeft() {
        if (backLeft == null) {
            backLeft = new SwerveModule(BACK_LEFT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.BACK_LEFT_DRIVE_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.BACK_LEFT_TURN_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.BACK_LEFT_ABSOLUTE_ENCODER), BACK_LEFT_DRIVE_INVERTED,
                    BACK_LEFT_CANCODER_INVERTED, BACK_LEFT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings);
        }
        return backLeft;
    }

    public static SwerveModule getBackRight() {
        if (backRight == null) {
            backRight = new SwerveModule(BACK_RIGHT_NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.BACK_RIGHT_DRIVE_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.BACK_RIGHT_TURN_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANcoder(RobotMap.CAN.BACK_RIGHT_ABSOLUTE_ENCODER), BACK_RIGHT_DRIVE_INVERTED,
                    BACK_RIGHT_CANCODER_INVERTED, BACK_RIGHT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings);
        }
        return backRight;
    }
}
