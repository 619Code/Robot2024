package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommandDown extends Command {
    
    private ClimbSubsystem sub;

    public ClimbCommandDown(ClimbSubsystem climb) {
        sub = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        sub.goDown();;
    }

}