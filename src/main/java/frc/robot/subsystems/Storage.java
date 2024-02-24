package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private final Ultrasonic ultrasonic;
    private final CANSparkMax sparkMax;
    private static Storage instance;

    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage(
                    NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.STORAGE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new Ultrasonic(RobotMap.DIO.STORAGE_ULTRASONIC_INPUT, RobotMap.DIO.STORAGE_ULTRASONIC_OUTPUT));
        }
        return instance;
    }

    private Storage(String namespaceName, CANSparkMax motor, Ultrasonic ultrasonic) {
        super(namespaceName, motor);
        this.ultrasonic = ultrasonic;
        Ultrasonic.setAutomaticMode(true);
        this.sparkMax = motor;
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        configureDashboard();
    }

    public boolean hasNote() {
        return ultrasonic.getRangeMM() < 60 && ultrasonic.getRangeMM() > 15;
    }

    public double getCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public void configureDashboard() {
        namespace.putCommand("move", new RunCommand(() -> motorController.set(-0.6)) {
            @Override
            public void end(boolean interrupted) {
                motorController.stopMotor();
            }
        });

        namespace.putNumber("current", sparkMax::getOutputCurrent);
        namespace.putNumber("distance", ultrasonic::getRangeMM);
        namespace.putBoolean("has note", this::hasNote);
    }
}
