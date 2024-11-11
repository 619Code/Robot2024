package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.Crashboard;

public class AutoSwitchBoardSub extends SubsystemBase {
    private DigitalInput diOnes, diTwos, diFours, diEights;
    private boolean single;

    public AutoSwitchBoardSub(boolean singleSwitch) {
        single = singleSwitch;

        if (singleSwitch) {
            //
            diOnes = new DigitalInput(Constants.SwitchboardConstants.diOnesPort);
        } else {
            diOnes = new DigitalInput(Constants.SwitchboardConstants.diOnesPort);
            diTwos = new DigitalInput(Constants.SwitchboardConstants.diTwosPort);            
            diFours = new DigitalInput(Constants.SwitchboardConstants.diFoursPort);
            diEights = new DigitalInput(Constants.SwitchboardConstants.diEightsPort);
        }

    }

    public boolean isPositionSourceSide() {
        return !diEights.get();
    }

    public boolean isPositionAmpSide() {
        return !diTwos.get();
    }

    public boolean isPositionForward() {
        return !diFours.get();
    }

    public boolean shouldTaxi() {
        return !diOnes.get();
    }

    public int getSwitchCombo() {
        int out = 0;
        if (diOnes.get()) out += 1;
        if (!single) {
            if (diTwos.get()) out += 2;
            if (diFours.get()) out += 4;
        }
        return out;
    }

    @Override
    public void periodic() {
        //Crashboard.toDashboard("auto switch", diOnes.get(), "Competition");
        Crashboard.toDashboard("Taxi Switch", shouldTaxi(), "Competition");
        Crashboard.toDashboard("Amp Side Switch", isPositionAmpSide(), "Competition");
        Crashboard.toDashboard("Forward Side Switch", isPositionForward(), "Competition");
        Crashboard.toDashboard("Source Side Switch", isPositionSourceSide(), "Competition");
    }

}
