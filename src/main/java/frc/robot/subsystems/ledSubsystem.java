package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ledSubsystem extends SubsystemBase {
    
    private CANdle led;

    public ledSubsystem() {
        this.led = new CANdle(Constants.CANdleid);
    }

    public void setColor(int r, int g, int b) {
        led.setLEDs(r, g, b);
    }

    public void setColor(int r, int g, int b, int w, int startlid, int count) {
        led.setLEDs(r, g, b, w, startlid, count);
    }

    @Override
    public void periodic() {
        //setColor(255, 255, 255);
    }

}