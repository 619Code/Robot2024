package frc.robot.commands;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.animations.CARAnimation;
import frc.robot.commands.animations.NightRider;
import frc.robot.commands.animations.FunnyAnimation;
import frc.robot.subsystems.ledSubsystem;


public class ledCommand extends Command {
    private ledSubsystem subsystem;
    

    private CARAnimation currentAnimation;

    public ledCommand(ledSubsystem subsystem) {
        this.subsystem = subsystem;
        
        System.out.println("This was called");

        currentAnimation = new NightRider(new Color(0, 0, 255), new Color(255, 0, 0), 50, 1, subsystem);

        addRequirements(subsystem);

    }

    @Override 
    public void initialize(){
      //  subsystem.setColor(0, 255, 0, 0, 20, 20);
    //    subsystem.setColor(255, 0, 0, 0, 0, 20);
     //   subsystem.setColor(0, 0, 255);


        //subsystem.StartAnimation();

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
