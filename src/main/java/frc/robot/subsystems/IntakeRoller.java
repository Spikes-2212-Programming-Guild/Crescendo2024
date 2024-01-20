package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class IntakeRoller extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "intake roller";
    private final Supplier<Double> speed = namespace.addConstantDouble("speed", 0);

    private static IntakeRoller instance;

    public static IntakeRoller getInstance() {
        if (instance == null) {
            instance = new IntakeRoller(
                    new CANSparkMax(RobotMap.CAN.INTAKE_ROLLER_SPARKMAX, CANSparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private IntakeRoller(CANSparkMax rollMotor) {
        super(NAMESPACE_NAME, rollMotor);
    }

    public void collect() {
        this.apply(speed.get());
    }
}
