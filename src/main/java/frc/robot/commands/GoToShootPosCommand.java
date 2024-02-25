package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HingeSubsystem;

public class GoToShootPosCommand extends Command {
    private HingeSubsystem subsystem;

    public GoToShootPosCommand(HingeSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setGoal(Constants.HingeConstants.kShootingAngle);
        subsystem.enable();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }

    @Override
    public boolean isFinished() {
        //return subsystem.isAtPosition(Constants.HingeConstants.kShootingAngle, 0 /*temp deadzone*/);
        return subsystem.getController().atGoal();
    }
}
