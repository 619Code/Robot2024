package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.TestHingeSubsystem;

public class TestHingeCommand extends Command {
    HingeSubsystem subsystem;
    double speed = 0;
    CommandXboxController controller;

    public TestHingeCommand(HingeSubsystem subsystem, CommandXboxController contr) {
        this.subsystem = subsystem;
        if (Math.abs(speed) < 0.1) speed = 0;
        controller = contr;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //subsystem.spinge(speed * .1);
        
    }

    @Override
    public void execute() {
        speed = controller.getLeftY();
        if (Math.abs(speed) < 0.1) speed = 0;
        subsystem.spinge(speed * 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopHinge();
    }

}
