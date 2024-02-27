package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private final DigitalInput ir;
    private final CANSparkMax sparkMax;
    private static Storage instance;
    private static boolean seen = false;

    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage(
                    NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.STORAGE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.DIO.STORAGE_IR));
        }
        return instance;
    }

    private Storage(String namespaceName, CANSparkMax motor, DigitalInput ir) {
        super(namespaceName, motor);
        this.ir = ir;
        this.sparkMax = motor;
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        configureDashboard();
    }

    public boolean seesNote() {
        return !ir.get();
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
        namespace.putBoolean("ir val", ir::get);
    }
}
