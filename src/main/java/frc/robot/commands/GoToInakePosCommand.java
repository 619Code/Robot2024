package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HingeSubsystem;

public class GoToInakePosCommand extends Command {
    private HingeSubsystem subsystem;

    public GoToInakePosCommand(HingeSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setGoal(Constants.HingeConstants.kIntakeAngle);
        subsystem.enable();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isAtPosition(Constants.HingeConstants.kIntakeAngle, 0 /*temp deadzone*/);
    }
}
