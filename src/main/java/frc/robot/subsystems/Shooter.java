package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.SpikesLogger;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.RobotMap;

public class Shooter extends DashboardedSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private final ShooterFlywheel leftFlywheel;
    private final ShooterFlywheel rightFlywheel;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public Shooter() {
        super(NAMESPACE_NAME);
        this.leftFlywheel = ShooterFlywheel.getLeftInstance();
        this.rightFlywheel = ShooterFlywheel.getRightInstance();
    }

    public ShooterFlywheel getLeftFlywheel() {
        return leftFlywheel;
    }

    public ShooterFlywheel getRightFlywheel() {
        return rightFlywheel;
    }

    @Override
    public void configureDashboard() {
    }
}
