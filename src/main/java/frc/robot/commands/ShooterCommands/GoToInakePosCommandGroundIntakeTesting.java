package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.HingeSubsystem;

public class GoToInakePosCommandGroundIntakeTesting extends Command {
    private HingeSubsystem subsystem;

    public GoToInakePosCommandGroundIntakeTesting(HingeSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setGoal(65);
        subsystem.enable();
        OurRobotState.currentArmPosition = ArmPosEnum.INTAKE;
        OurRobotState.isClimbing = false;

    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }

    @Override
    public boolean isFinished() {
        //return subsystem.isAtPosition(Constants.HingeConstants.kIntakeAngle, 0 /*temp deadzone*/);
        //return subsystem.getController().atGoal();
        return false;
    }
}
