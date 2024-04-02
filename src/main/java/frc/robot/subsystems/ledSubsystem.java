package frc.robot.subsystems;

//supercalifragilisticexpialidocious
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ledSubsystem extends SubsystemBase {

    private CANdle candle;
    private Animation currentAnimation = null;

    public ledSubsystem() {
        this.candle = new CANdle(Constants.LEDConstants.CANdleid);
        // currentAnimation = new ColorFlowAnimation(255, 0, 0, 0, 0.8, Constants.LEDConstants.ledCount,
        //         Direction.Forward);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.v5Enabled = true;
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        this.candle.configAllSettings(configAll, 100);

    }

    // public void StartAnimation() {
    //     //candle.animate(currentAnimation);
    // }

    // public void StopAnimation() {
    //     //candle.animate(null);
    //     // candle.clearAnimation(0);
    // }

    public void setColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setColor(int r, int g, int b, int w, int startlid, int count) {
        candle.setLEDs(r, g, b, w, startlid, count);
    }

    public void TurnOffLEDs() {
        candle.setLEDs(0, 0, 0);
    }
}