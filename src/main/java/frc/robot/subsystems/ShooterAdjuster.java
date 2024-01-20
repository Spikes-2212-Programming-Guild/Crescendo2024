package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterAdjuster extends SparkGenericSubsystem {

    private static String NAMESPACE_NAME;
    private static final double ENCODER_OFFSET = 0;

    private final DutyCycleEncoder absoluteEncoder;

    public ShooterAdjuster(CANSparkMax left, CANSparkMax right, DutyCycleEncoder absoluteEncoder) {
        super(NAMESPACE_NAME, left, right);
        this.absoluteEncoder = absoluteEncoder;
    }

    public double getAngle() {
        return absoluteEncoder.get() * 360 + ENCODER_OFFSET;
    }
}
