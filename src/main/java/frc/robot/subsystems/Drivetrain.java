package frc.robot.subsystems;


import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.geometry.Pose2d;

public class Drivetrain extends DashboardedSubsystem {
    public Drivetrain(Namespace namespace) {
        super(namespace);
    }

    public PIDSettings getPIDSettings(){
        return null;
    }
    public FeedForwardSettings getFeedForwardSettings(){
        return null;
    }

    public Pose2d getPose(){
        return null;
    }
    public double getAngle() {
        return 0;
    }
    // using mcs
    @Override
    public void configureDashboard() {

    }
}
