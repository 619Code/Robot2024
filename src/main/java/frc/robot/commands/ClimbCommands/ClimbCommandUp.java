package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommandUp extends Command {
    
    private ClimbSubsystem sub;

    public ClimbCommandUp(ClimbSubsystem climb) {
        sub = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        sub.goUp();
    }

}