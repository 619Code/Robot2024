package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private DoubleSolenoid leftArm, rightArm;

    public ClimbSubsystem() {

        leftArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimbConstants.kLeftArmForwardPort, Constants.ClimbConstants.kLeftArmBackwardPort);
        rightArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimbConstants.kRightArmForwardPort, Constants.ClimbConstants.kRightArmBackwardPort);

    }

    public void reverse() {

        leftArm.toggle();
        rightArm.toggle();
        
    }

}
