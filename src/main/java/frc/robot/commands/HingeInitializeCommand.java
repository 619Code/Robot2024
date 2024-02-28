package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.TestHingeSubsystem;

public class HingeInitializeCommand extends Command {
    private HingeSubsystem subsystem;

    public HingeInitializeCommand(HingeSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spinge(-.07);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopHinge();
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAbsoluteDegrees() <= Constants.HingeConstants.kMinAngle+1) {
            subsystem.resetRelativeEncoders();
            subsystem.SetRelativeEncoderSoftLimits(Constants.HingeConstants.bottomEncoderSoftLimit, Constants.HingeConstants.topEncoderSoftLimit);
            OurRobotState.ArmInitialized = true;
            return true;
        }
        else {
            return false;
        }
    }
}
