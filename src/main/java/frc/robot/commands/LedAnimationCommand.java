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
import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.ledSubsystem;

public class LedAnimationCommand extends Command {
    private ledSubsystem subsystem;
    private ArrayList<CARAnimation> animations;

    int sideLedCount;
    int initialOffset = 8;
    boolean inMatch = false;

    boolean isEndGame = false;

    public LedAnimationCommand(ledSubsystem subsystem) {
        this.subsystem = subsystem;
        sideLedCount = 39;
        animations = new ArrayList<CARAnimation>();
        animations.add(new NightRider(new Color(0, 0, 255), new Color(255, 64, 0), 6, 1, sideLedCount, initialOffset,
                false, subsystem));
        animations.add(new NightRider(new Color(0, 0, 255), new Color(255, 64, 0), 6, 1, sideLedCount,
                sideLedCount + initialOffset + 1, true, subsystem));

        addRequirements(subsystem);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {

        for (CARAnimation animation : animations)
            animation.start();

    }

    @Override
    public void execute() {

        Crashboard.toDashboard("Match Time", DriverStation.getMatchTime(), "Compitition");
        if (OurRobotState.isEnabled) {
            if (!DriverStation.isAutonomous() && DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() != -1) {
                

                isEndGame = true;

                if(OurRobotState.hasNote){

                    int topLength = (int)sideLedCount/2;
                    int middleLength = sideLedCount;

                    subsystem.setColor(0, 255, 0, 0, 8, topLength + 1); 

                    subsystem.setColor(255, 75, 0, 0, 8 + topLength+1, middleLength); 

                    subsystem.setColor(0, 255, 0, 0, 8 + topLength + middleLength + 1, topLength);

                }else{

                    subsystem.setColor(255, 75, 0);

                }
            }
            else if (OurRobotState.hasNote) {

                subsystem.setColor(0, 255, 0);

            }
            else {
                subsystem.setColor(0, 0, 255);
                isEndGame = false;
            }
        }
        else 
        {
            for (CARAnimation animation : animations)
                animation.update();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // subsystem.StopAnimation();
        // subsystem.TurnOffLEDs();
    }
}