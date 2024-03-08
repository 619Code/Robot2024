package frc.robot.commands.Unused;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ledSubsystem;


public class ledCommand extends Command {
    private ledSubsystem subsystem;
    private final XboxController controller;

    public ledCommand(ledSubsystem subsystem, XboxController controller) {
        this.subsystem = subsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        if (controller.getAButtonPressed()){
            subsystem.setColor(100, 255, 70);
        }
        else {
            subsystem.setColor(255, 255, 255);
        }
    }
    

}