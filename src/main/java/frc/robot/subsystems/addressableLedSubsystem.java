package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class addressableLedSubsystem extends SubsystemBase {
    


    private AddressableLED ledStrip;
    private AddressableLEDBuffer buffer;


    public addressableLedSubsystem() {
        
        ledStrip = new AddressableLED(9);

        buffer = new AddressableLEDBuffer(108);
        ledStrip.setLength(buffer.getLength());

        for (int i = 0; i < buffer.getLength(); i++) {
        //   Sets the specified LED to the RGB values for red
            buffer.setRGB(i, 255, 0, 0);
        }

        ledStrip.setData(buffer);

        ledStrip.setData(buffer);


        ledStrip.start();

    }
}