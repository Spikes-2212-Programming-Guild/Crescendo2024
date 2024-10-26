package frc.robot.subsystems;

import com.spikes2212.command.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

/**
 * A class representing a single climber which uses a double solenoid.
 */
public class Climber extends DoubleSolenoidSubsystem {

    private static final String LEFT_NAMESPACE_NAME = "left climber";
    private static final String RIGHT_NAMESPACE_NAME = "right climber";

    private static final boolean LEFT_INVERTED = false;
    private static final boolean RIGHT_INVERTED = false;

    // the robot was supposed to have 2 climbers, one on each side
    private static Climber leftInstance;
    private static Climber rightInstance;

    public static Climber getLeftInstance() {
        if (leftInstance == null) {
            leftInstance = new Climber(
                    LEFT_NAMESPACE_NAME,
                    new DoubleSolenoid(
                            PneumaticsModuleType.REVPH,
                            RobotMap.RPH.LEFT_CLIMBER_FORWARD,
                            RobotMap.RPH.LEFT_CLIMBER_BACKWARD),
                    LEFT_INVERTED
            );
        }
        return leftInstance;
    }

    public static Climber getRightInstance() {
        if (rightInstance == null) {
            rightInstance = new Climber(
                    RIGHT_NAMESPACE_NAME,
                    new DoubleSolenoid(
                            PneumaticsModuleType.REVPH,
                            RobotMap.RPH.RIGHT_CLIMBER_FORWARD,
                            RobotMap.RPH.RIGHT_CLIMBER_BACKWARD),
                    RIGHT_INVERTED
            );
        }
        return rightInstance;
    }

    private Climber(String namespaceName, DoubleSolenoid doubleSolenoid, boolean inverted) {
        super(namespaceName, doubleSolenoid, inverted);
    }
}
