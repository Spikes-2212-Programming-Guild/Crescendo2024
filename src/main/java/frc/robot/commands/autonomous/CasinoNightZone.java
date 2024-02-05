package frc.robot.commands.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CasinoNightZone extends BasePathAuto {

    public CasinoNightZone(Drivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public Command getCommand() {
        return new PathPlannerAuto("CasinoNightZone");
    }
}
