package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HingeSubsystem;

public class HingeInitializeCommand extends Command {
    private HingeSubsystem subsystem;

    public HingeInitializeCommand(HingeSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spinge(-.01);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopHinge();
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAbsoluteDegrees() <= Constants.HingeConstants.kMinAngle) {
            return true;
        }
        else {
            return false;
        }
    }
}
