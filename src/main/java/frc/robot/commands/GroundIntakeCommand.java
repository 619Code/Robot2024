package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class GroundIntakeCommand extends Command {

    private GroundIntakeSubsystem sub;

    public GroundIntakeCommand(GroundIntakeSubsystem system) {

        sub = system;

        addRequirements(system);
    }

    @Override
    public void end(boolean interrupted) {
        sub.spintake(0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        sub.spintake(Constants.GroundIntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
