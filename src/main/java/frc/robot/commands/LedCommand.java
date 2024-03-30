package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Animations.CARAnimation;
import frc.robot.commands.Animations.NightRider;
import frc.robot.subsystems.ledSubsystem;


public class LedCommand extends Command {
    private ledSubsystem subsystem;
    private CARAnimation currentAnimation;

    public LedCommand(ledSubsystem subsystem) {
        this.subsystem = subsystem;
        
        //System.out.println("This was called");
        currentAnimation = new NightRider(new Color(0, 0, 255), new Color(255, 0, 0), 50, 1, Constants.LEDConstants.ledCount, subsystem);

        addRequirements(subsystem);
    }

    @Override 
    public void initialize(){
         currentAnimation.start();
    }


    @Override
    public void execute(){
        currentAnimation.update();
    }
    
    @Override 
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        subsystem.StopAnimation();
        subsystem.TurnOffLEDs();
    }
}