package frc.robot.commands;


import java.util.ArrayList;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.commands.Animations.CARAnimation;
import frc.robot.commands.Animations.NightRider;
import frc.robot.subsystems.ledSubsystem;


public class LedAnimationCommand extends Command {
    private ledSubsystem subsystem;
    // private CARAnimation animationOne;
    // private CARAnimation animationTwo;
    private ArrayList<CARAnimation> animations;

    int sideLedCount;
    int initialOffset = 8;
    boolean inMatch = false;

    boolean isEnabled = false;
    boolean isEndGame = false;

    public LedAnimationCommand(ledSubsystem subsystem) {
        this.subsystem = subsystem;
        sideLedCount = 39;
        //System.out.println("This was called");
        animations = new ArrayList<CARAnimation>();
        animations.add(new NightRider(new Color(0, 0, 255), new Color(255, 64, 0), 6, 1, sideLedCount,initialOffset, false, subsystem));
        animations.add(new NightRider(new Color(0, 0, 255), new Color(255, 64, 0), 6, 1, sideLedCount, sideLedCount+initialOffset+1, true, subsystem));

        addRequirements(subsystem);
    }

    @Override
    public boolean runsWhenDisabled()
    {
        return true;
    }

    @Override 
    public void initialize(){

        for (CARAnimation animation:animations)
            animation.start();

    }


    @Override
    public void execute(){

        if (isEnabled != OurRobotState.isEnabled)
        {
            isEnabled = OurRobotState.isEnabled;
            if (isEnabled)
            {
                // Blue
                subsystem.setColor(0, 0, 255);
            }
        }
        else
        {
            isEnabled = OurRobotState.isEnabled;
        }

        if (!isEnabled)
        {
            for (CARAnimation animation:animations)
                animation.update();
        }

        if (isEnabled && !DriverStation.isAutonomous() && DriverStation.getMatchTime() <= 30)
        {
            isEndGame = true;
            subsystem.setColor(255, 75, 0);
        }
        else
        {

            isEndGame = false;
        }
    }
    
    @Override 
    public boolean isFinished(){
        return false;
    }



    @Override
    public void end(boolean interrupted){
        //subsystem.StopAnimation();
        //subsystem.TurnOffLEDs();
    }
}