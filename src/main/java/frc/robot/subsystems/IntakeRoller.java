package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.RobotMap;

/**
 * A class which controls the intake's wheels, using one NEO 550.
 */
public class IntakeRoller extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "intake roller";

    private static final int CURRENT_LIMIT = 20;
    private static final int STATUS_0_PERIODIC_FRAME = 100;
    private static final int STATUS_1_PERIODIC_FRAME = 500;
    private static final int STATUS_2_PERIODIC_FRAME = 500;

    private static IntakeRoller instance;

    public static IntakeRoller getInstance() {
        if (instance == null) {
            instance = new IntakeRoller(
                    new CANSparkMax(RobotMap.CAN.INTAKE_ROLLER_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private IntakeRoller(CANSparkMax rollMotor) {
        super(NAMESPACE_NAME, rollMotor);
        rollMotor.restoreFactoryDefaults();
        rollMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, STATUS_0_PERIODIC_FRAME);
        rollMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, STATUS_1_PERIODIC_FRAME);
        rollMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, STATUS_2_PERIODIC_FRAME);
        rollMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !Storage.getInstance().seesNote();
    }

    @Override
    public void configureDashboard() {
        namespace.putCommand("move", new MoveGenericSubsystem(this, 0.65));
    }
}
