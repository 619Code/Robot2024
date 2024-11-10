package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;

    public GroundIntakeSubsystem() {
        
        intakeMotor = new CANSparkMax(Constants.GroundIntakeConstants.intakeMotorId, CANSparkLowLevel.MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        
    }

    public void spintake(double speed) {
        intakeMotor.set(speed);
    }

}
