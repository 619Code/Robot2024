package frc.robot.commands.animations;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.ledSubsystem;

public class NightRider implements CARAnimation{
    
    private ledSubsystem subsystem;
    private Color backgroundColor, foregroundColor;
    private int lengthOfForeground;
    private int dir = 1;
    private int speed;
    
    public NightRider(Color _backgroundColor, Color _foregroundColor, int _lengthOfForeground, int _speed, ledSubsystem _subsystem){
        subsystem = _subsystem;
        backgroundColor = _backgroundColor;
        foregroundColor = _foregroundColor;
        lengthOfForeground = _lengthOfForeground;
        speed = _speed;
    }
    
    public void start(){


        
    }

    private int pos = 0;

    public void update(){
        
        //  Background   
        subsystem.setColor((int)backgroundColor.red * 255, (int)backgroundColor.green * 255, (int)backgroundColor.blue * 255, 0, 0, pos - 1);
        subsystem.setColor((int)backgroundColor.red * 255, (int)backgroundColor.green * 255, (int)backgroundColor.blue * 255, 0, pos + lengthOfForeground + 1, Constants.LEDConstants.ledCount);
        // Foregroung
        subsystem.setColor((int)foregroundColor.red *255, (int)foregroundColor.green * 255, (int)foregroundColor.blue * 255, 0, pos, lengthOfForeground);
        
        pos += dir * speed;
        if(pos >= Constants.LEDConstants.ledCount - lengthOfForeground){
            pos = Constants.LEDConstants.ledCount - lengthOfForeground;
            dir *= -1;
        
        if(pos < 0){
            pos = 0;
            dir *= -1;
        }
    }


}
}