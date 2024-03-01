package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
<<<<<<< HEAD
import com.revrobotics.RelativeEncoder;
=======
>>>>>>> main
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
<<<<<<< HEAD
import frc.robot.helpers.Crashboard;
=======
>>>>>>> main

public class ManipulatorSubsystem extends SubsystemBase {

    public final CANSparkMax intakeLeader;
    public final CANSparkMax shooterLeader;

    public final DigitalInput intakeProximitySensor;
<<<<<<< HEAD

    public final RelativeEncoder shooterEncoder;
=======
    public final DigitalInput middleProximitySensor;
>>>>>>> main

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
<<<<<<< HEAD
        shooterLeader.setInverted(Constants.ManipulatorConstants.kShooterLeaderInverted);

        intakeProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kIntakeSensorPort);

        shooterEncoder = this.shooterLeader.getEncoder();

    }

    @Override
    public void periodic() {
        
        Crashboard.toDashboard("Sensor value: ", intakeProximitySensor.get(), "Manipulator");

        

    }

    public double GetShooterVelocity(){

        return shooterEncoder.getVelocity();
=======
        shooterLeader.setInverted(Constants.ManipulatorConstants.kInakeLeaderInverted);

        intakeProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kIntakeSensorPort);
        middleProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kMiddleSensorPort);

>>>>>>> main

    }

    public void spintake(double speed) {
        intakeLeader.set(speed);
    }

    public void spinShooter(double speed) {
        shooterLeader.set(speed);
    }

    public boolean intakeTrigged() {
<<<<<<< HEAD
        return !intakeProximitySensor.get();
    }

    public void stopIntake(){
        intakeLeader.stopMotor();
    }

    public void stopShooter(){
        shooterLeader.stopMotor();
=======
        return intakeProximitySensor.get();
    }

    public boolean middleTriggered() {
        return middleProximitySensor.get();
    }

    public void stopIntake(){
        intakeLeader.stopMotor();;
    }

    public void stopShooter(){
        shooterLeader.stopMotor();;
>>>>>>> main
    }

    public void stopAll(){
        intakeLeader.stopMotor();
        shooterLeader.stopMotor();
<<<<<<< HEAD
    }  
}
=======
    }


    
    
}
>>>>>>> main
