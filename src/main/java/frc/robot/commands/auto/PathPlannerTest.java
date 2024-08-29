package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drivetrain;

public class PathPlannerTest extends FollowPathCommand {

    private static final RootNamespace root = new RootNamespace("path planner test");
    private static final PIDSettings translationSettings = root.addPIDNamespace("translation", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings rotationSettings = root.addPIDNamespace("rotation", PIDSettings.EMPTY_PID_SETTINGS);

    public PathPlannerTest(PathPlannerPath path, Drivetrain drivetrain) {
        super(path, drivetrain::getPose, drivetrain::getRobotRelativeSpeeds,
                speeds -> drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, false, true),
                new PPHolonomicDriveController(
                        new PIDConstants(translationSettings.getkP(),
                                translationSettings.getkI(), translationSettings.getkD()),
                        new PIDConstants(rotationSettings.getkP(),
                                rotationSettings.getkI(), rotationSettings.getkD()),
                        Drivetrain.MAX_SPEED_METERS_PER_SECONDS,
                        Drivetrain.FRONT_RIGHT_WHEEL_POSITION.getNorm()
                ),
                new ReplanningConfig(),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                drivetrain
        );
    }
}
