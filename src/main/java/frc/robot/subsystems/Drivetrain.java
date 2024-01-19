package frc.robot.subsystems;


import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.geometry.Pose2d;

public class Drivetrain extends DashboardedSubsystem {
    public Drivetrain(Namespace namespace) {
        super(namespace);
    }

    public Pose2d getPose(){
        Pose2d pose = new Pose2d();
        return pose;
    }

    @Override
    public void configureDashboard() {

    }
}
