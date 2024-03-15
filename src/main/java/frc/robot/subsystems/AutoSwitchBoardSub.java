package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutoSwitchBoardSub extends SubsystemBase {
    private DigitalInput diOnes, diTwos, diFours;

    public AutoSwitchBoardSub() {

        diOnes = new DigitalInput(Constants.SwitchboardConstants.diOnesPort);
        diTwos = new DigitalInput(Constants.SwitchboardConstants.diTwosPort);
        diFours = new DigitalInput(Constants.SwitchboardConstants.diFoursPort);

    }

    public int getSwitchCombo() {
        int out = 0;
        if (diOnes.get()) out += 1;
        if (diTwos.get()) out += 2;
        if (diFours.get()) out += 4;
        return out;
    }

}
