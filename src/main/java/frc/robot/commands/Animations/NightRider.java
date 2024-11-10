package frc.robot.commands.Animations;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ledSubsystem;

public class NightRider implements CARAnimation {

    private ledSubsystem subsystem;
    private Color backgroundColor, foregroundColor;
    private int lengthOfForeground;
    private int dir = 1;
    private int speed;
    private int ledCount;
    private int ledOffset;
    private boolean reversed;

    public NightRider(Color _backgroundColor, Color _foregroundColor, int _lengthOfForeground, int _speed,
            int _ledCount, int _ledOffset, boolean _reversed, ledSubsystem _subsystem) {
        subsystem = _subsystem;
        backgroundColor = _backgroundColor;
        foregroundColor = _foregroundColor;
        lengthOfForeground = _lengthOfForeground;
        speed = _speed;
        ledCount = _ledCount;
        ledOffset = _ledOffset;
        this.reversed = _reversed;
    }

    public void start() {
        if (reversed)
        { 
            this.pos = ledCount - lengthOfForeground;
        }
    }

    private int pos = 0;

    public void update(){
        
        //  Background   
        subsystem.setColor(ConvertColor(backgroundColor.red), ConvertColor(backgroundColor.green), ConvertColor(backgroundColor.blue), 0, this.getOffset(0), this.getOffset(pos - 1));
        subsystem.setColor(ConvertColor(backgroundColor.red), ConvertColor(backgroundColor.green), ConvertColor(backgroundColor.blue), 0, this.getOffset(pos + lengthOfForeground + 1), this.getOffset(ledCount));
        // Foregroung
        subsystem.setColor(ConvertColor(foregroundColor.red), ConvertColor(foregroundColor.green), ConvertColor(foregroundColor.blue), 0, this.getOffset(pos), lengthOfForeground);
        
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

    private int getOffset(int value) {
        return value + this.ledOffset;
    }

    private int ConvertColor(double colorValue) {
        return (int)((double)255 * colorValue);
    }
}
