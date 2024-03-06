package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;

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
        configureDashboard();
    }

    public ShooterFlywheel getLeftFlywheel() {
        return leftFlywheel;
    }

    public ShooterFlywheel getRightFlywheel() {
        return rightFlywheel;
    }

    public void stop() {
        leftFlywheel.finish();
        rightFlywheel.finish();
        leftFlywheel.stop();
        rightFlywheel.stop();
    }

    @Override
    public void configureDashboard() {
        namespace.putRunnable("stop", this::stop);
    }
}
