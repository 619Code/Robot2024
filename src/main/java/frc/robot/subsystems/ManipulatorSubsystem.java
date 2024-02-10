package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {

    public final CANSparkMax intakeLeader;
    public final CANSparkMax shooterLeader;

    public final DigitalInput intakeProximitySensor;
    public final DigitalInput middleProximitySensor;

    public ManipulatorSubsystem() {
        intakeLeader = new CANSparkMax(Constants.ManipulatorConstants.kIntakeLeaderPort, MotorType.kBrushless);
        intakeLeader.restoreFactoryDefaults();
        intakeLeader.setIdleMode(IdleMode.kBrake);
        intakeLeader.setSmartCurrentLimit(35);
        intakeLeader.setInverted(Constants.ManipulatorConstants.kInakeLeaderInverted);

        shooterLeader = new CANSparkMax(Constants.ManipulatorConstants.kShooterLeaderPort, MotorType.kBrushless);
        shooterLeader.restoreFactoryDefaults();
        shooterLeader.setIdleMode(IdleMode.kBrake);        
        shooterLeader.setSmartCurrentLimit(35);
        shooterLeader.setInverted(Constants.ManipulatorConstants.kInakeLeaderInverted);

        intakeProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kIntakeSensorPort);
        middleProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kMiddleSensorPort);


    }

    public void spintake(double speed) {
        intakeLeader.set(speed);
    }

    public void spinShooter(double speed) {
        shooterLeader.set(speed);
    }

    public boolean intakeTrigged() {
        return intakeProximitySensor.get();
    }

    public boolean middleTriggered() {
        return middleProximitySensor.get();
    }


    
    
}
