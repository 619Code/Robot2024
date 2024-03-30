package frc.robot.commands.Animations;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.commands.Unused.ledCommand;
import frc.robot.subsystems.ledSubsystem;

public class NightRider implements CARAnimation {

    private ledSubsystem subsystem;
    private Color backgroundColor, foregroundColor;
    private int lengthOfForeground;
    private int dir = 1;
    private int speed;
    private int ledCount;

    public NightRider(Color _backgroundColor, Color _foregroundColor, int _lengthOfForeground, int _speed,
            int _ledCount, ledSubsystem _subsystem) {
        subsystem = _subsystem;
        backgroundColor = _backgroundColor;
        foregroundColor = _foregroundColor;
        lengthOfForeground = _lengthOfForeground;
        speed = _speed;
        ledCount = _ledCount;
    }

    public void start() {
    }

    private int pos = 0;

    public void update(){
        
        //  Background   
        subsystem.setColor((int)backgroundColor.red, (int)backgroundColor.green, (int)backgroundColor.blue, 0, 0, pos - 1);
        subsystem.setColor((int)backgroundColor.red, (int)backgroundColor.green, (int)backgroundColor.blue, 0, pos + lengthOfForeground + 1, ledCount);
        // Foregroung
        subsystem.setColor((int)foregroundColor.red, (int)foregroundColor.green, (int)foregroundColor.blue, 0, pos, lengthOfForeground);
        
        pos += dir * speed;
        if(pos >= ledCount - lengthOfForeground){
            pos = ledCount - lengthOfForeground;
            dir *= -1;
        }
        
        if(pos < 0){
            pos = 0;
            dir *= -1;
        }
    }
}
