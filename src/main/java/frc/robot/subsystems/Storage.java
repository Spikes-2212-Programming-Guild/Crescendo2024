package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

/**
 * A class which represents the storage, which uses a motor to move a note to the shooter and an infrared sensor which
 * returns whether there is a note inside
 */
public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private static final int STATUS_0_PERIODIC_FRAME = 100;
    private static final int STATUS_1_PERIODIC_FRAME = 500;
    private static final int STATUS_2_PERIODIC_FRAME = 500;

    private final DigitalInput infrared;
    private final CANSparkMax sparkMax;
    private boolean seen = false;

    private static Storage instance;

    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage(
                    NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.STORAGE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.DIO.STORAGE_IR));
        }
        return instance;
    }

    private Storage(String namespaceName, CANSparkMax motor, DigitalInput infrared) {
        super(namespaceName, motor);
        this.infrared = infrared;
        this.sparkMax = motor;
        // an issue which we've had is the can usage being too high. to prevent that we slowed down the periodic frame
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, STATUS_0_PERIODIC_FRAME);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, STATUS_1_PERIODIC_FRAME);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, STATUS_2_PERIODIC_FRAME);
        configureDashboard();
    }

    public boolean seesNote() {
        return !infrared.get();
    }

    public boolean cantMove() {
        if (seesNote()) {
            seen = true;
        }
        if (seen && !seesNote()) {
            seen = false;
            return true;
        }
        return false;
    }

    public double getCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public void configureDashboard() {
        namespace.putCommand("move", new MoveGenericSubsystem(this, -0.4).until(this::cantMove));
        namespace.putCommand("just move", new MoveGenericSubsystem(this, -0.6));

        namespace.putNumber("current", sparkMax::getOutputCurrent);
        namespace.putBoolean("sees note", this::seesNote);
        namespace.putBoolean("ir val", infrared::get);
    }
}
