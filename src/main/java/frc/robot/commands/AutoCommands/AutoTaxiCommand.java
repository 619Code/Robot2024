package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTaxiCommand extends Command {

    private SwerveSubsystem sub;
    private double startingOrientation = 0;
    
    public AutoTaxiCommand(SwerveSubsystem system) {
        sub = system;
        addRequirements(system);
    }

    @Override
    public void initialize() {
        startingOrientation = sub.getHeading();
        if (Math.abs(startingOrientation) < 15) { startingOrientation = 0;}
    }

    @Override
    public void execute() {
        //
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        sub.stopModules();
    }

    

}