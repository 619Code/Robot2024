package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ClimbWithArmCommandDown extends Command {
    private HingeSubsystem system;
    private double highPos = 131;

    public ClimbWithArmCommandDown(HingeSubsystem sub) {
        system = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        OurRobotState.isClimbing = true;
        OurRobotState.currentArmPosition = ArmPosEnum.CLIMB;
        //system.setGoal(highPos);
        //system.enable();
        system.disable();
    }

    @Override
    public void end(boolean interrupted) {
        system.disable();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
