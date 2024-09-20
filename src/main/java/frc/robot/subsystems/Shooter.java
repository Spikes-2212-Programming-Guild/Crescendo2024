package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;

/**
 * A class which controls the shooter's speed using 2 {@link ShooterFlywheel}s: one on each side.
 */
public class Shooter extends DashboardedSubsystem {

    private static final String NAMESPACE_NAME = "shooter";

    private final ShooterFlywheel leftFlywheel;
    private final ShooterFlywheel rightFlywheel;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter(ShooterFlywheel.getLeftInstance(), ShooterFlywheel.getRightInstance());
        }
        return instance;
    }

    public Shooter(ShooterFlywheel leftFlywheel, ShooterFlywheel rightFlywheel) {
        super(NAMESPACE_NAME);
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        configureDashboard();
    }

    public ShooterFlywheel getLeftFlywheel() {
        return leftFlywheel;
    }

    public ShooterFlywheel getRightFlywheel() {
        return rightFlywheel;
    }

    public void stop() {
        leftFlywheel.stop();
        rightFlywheel.stop();
    }

    @Override
    public void configureDashboard() {
        namespace.putRunnable("stop", this::stop);
    }
}
