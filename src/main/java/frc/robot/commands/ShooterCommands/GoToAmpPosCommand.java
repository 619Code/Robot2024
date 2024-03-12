package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.HingeSubsystem;

public class GoToAmpPosCommand extends Command {
    private HingeSubsystem subsystem;

    public GoToAmpPosCommand(HingeSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setGoal(Constants.HingeConstants.kAmpAngle);
        subsystem.enable();
        OurRobotState.currentArmPosition = ArmPosEnum.AMP;
        OurRobotState.isClimbing = false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }

    @Override
    public boolean isFinished() {
        //return subsystem.isAtPosition(Constants.HingeConstants.kAmpAngle, 0 /*temp deadzone*/);
        return subsystem.getController().atGoal();
    }
}
