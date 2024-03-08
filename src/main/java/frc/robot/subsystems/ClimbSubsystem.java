package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private DoubleSolenoid leftArm, rightArm;

    public ClimbSubsystem() {

        leftArm = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, Constants.ClimbConstants.kLeftArmForwardPort, Constants.ClimbConstants.kLeftArmBackwardPort);
        rightArm = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, Constants.ClimbConstants.kRightArmForwardPort, Constants.ClimbConstants.kRightArmBackwardPort);


        leftArm.set(DoubleSolenoid.Value.kReverse);
        rightArm.set(DoubleSolenoid.Value.kReverse);

    }

    public void reverse() {

        leftArm.toggle();
        rightArm.toggle();
        
    }

    @Override
  public void periodic() {
    System.out.println("First: " + leftArm.get() + ", Second: " + rightArm.get());
  }

}
