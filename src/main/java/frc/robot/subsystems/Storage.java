package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private final DigitalInput limit;
    private static Storage instance;

    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage (
                    NAMESPACE_NAME,
                    new CANSparkMax(RobotMap.CAN.STORAGE_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.DIO.STORAGE_LIMIT)
            );
        }
        return instance;
    }

    private Storage(String namespaceName, MotorController motor, DigitalInput limit) {
        super(namespaceName, motor);
        this.limit = limit;
        configureDashboard();
    }

    public boolean hasNote() {
        return limit.get();
    }

    @Override
    public void configureDashboard() {
        namespace.putCommand("move", new RunCommand(() -> motorController.set(-0.6)) {
            @Override
            public void end(boolean interrupted) {
                motorController.stopMotor();
            }
        });
    }
}
