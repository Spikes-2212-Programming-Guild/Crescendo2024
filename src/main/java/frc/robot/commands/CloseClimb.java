package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class CloseClimb extends ParallelCommandGroup {

    public CloseClimb(Climber leftClimber, Climber rightClimber) {
        addCommands(leftClimber.closeCommand(), rightClimber.closeCommand());
    }
}
