package frc.robot.subsystems;


import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Drivetrain extends DashboardedSubsystem {
    public Drivetrain(Namespace namespace) {
        super(namespace);
    }
    public PIDSettings pidSettings(){
        return null;
    }
    public FeedForwardSettings feedForwardSettings(){
        return null;
    }

    public Pose2d getPose(){
        Pose2d pose = new Pose2d();
        return pose;
    }
    public double rotation(){
        return getPose().getRotation().getDegrees();
    }
    // using mcs
    @Override
    public void configureDashboard() {

    }
}
