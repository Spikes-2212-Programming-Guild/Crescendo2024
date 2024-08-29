package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap;
import frc.robot.commands.CloseIntake;
import frc.robot.commands.OpenIntake;

import java.util.function.Supplier;

/**
 * A class which represents the intake's placing subsystem, using 2 NEOs; one of each side.
 */
public class IntakePlacer extends SparkGenericSubsystem {

    private static final String NAMESPACE_NAME = "intake placer";

    private final PIDSettings pidSettings = namespace.addPIDNamespace("", new PIDSettings(0.7, 0.0, 1000));
    private final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("",
            FeedForwardSettings.EMPTY_FFSETTINGS);

    private static IntakePlacer instance;

    public static IntakePlacer getInstance() {
        if (instance == null) {
            instance = new IntakePlacer(
                    new CANSparkMax(RobotMap.CAN.INTAKE_PLACER_LEFT_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.INTAKE_PLACER_RIGHT_SPARK_MAX, CANSparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private IntakePlacer(CANSparkMax left, CANSparkMax right) {
        super(NAMESPACE_NAME, left, right);
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();
        left.setIdleMode(CANSparkBase.IdleMode.kBrake);
        right.setIdleMode(CANSparkBase.IdleMode.kBrake);
        // left is master, right is slaves.get(0)
        slaves.get(0).follow(master, true);
        master.getEncoder().setPosition(0);
        slaves.get(0).getEncoder().setPosition(0);
        configureDashboard();
    }

    public PIDSettings getPIDSettings() {
        return pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        master.setIdleMode(CANSparkBase.IdleMode.kBrake);
        slaves.get(0).setIdleMode(CANSparkBase.IdleMode.kBrake);
        slaves.get(0).follow(master, true);
    }

    public double getPosition() {
        return master.getEncoder().getPosition();
    }

    public void resetPosition() {
        master.getEncoder().setPosition(0);
    }

    @Override
    public void configureDashboard() {
        namespace.putCommand("open", new OpenIntake(this));
        namespace.putCommand("close", new CloseIntake(this));
        namespace.putRunnable("reset position", () -> master.getEncoder().setPosition(0));
        namespace.putNumber("position", this::getPosition);
        namespace.putBoolean("master break", () -> master.getIdleMode() == CANSparkBase.IdleMode.kBrake);
        namespace.putBoolean("slave break", () -> slaves.get(0).getIdleMode() == CANSparkBase.IdleMode.kBrake);
        namespace.putRunnable("set position 0", () -> master.getEncoder().setPosition(0));
    }
}
