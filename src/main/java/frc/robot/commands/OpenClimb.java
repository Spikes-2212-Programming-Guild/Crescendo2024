package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class OpenClimb extends ParallelCommandGroup {

    public OpenClimb(Climber leftClimber, Climber rightClimber) {
        addCommands(leftClimber.openCommand(), rightClimber.openCommand());
    }
}
