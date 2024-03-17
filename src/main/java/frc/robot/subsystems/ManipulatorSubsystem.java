package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.Crashboard;

public class ManipulatorSubsystem extends SubsystemBase {

    public final CANSparkMax intakeLeader;
    public final CANSparkMax shooterLeader;

    public final DigitalInput intakeProximitySensor;

    public final RelativeEncoder shooterEncoder;

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

    }

    public void spintake(double speed) {
        intakeLeader.set(speed);
    }

    public void spintakeVoltage(double voltage) {
        intakeLeader.setVoltage(voltage);
    }

    public void spinShooterVoltage(double voltage) {
        shooterLeader.setVoltage(voltage);
    }

    public void spinShooter(double speed) {
        shooterLeader.set(speed);
    }

    public boolean intakeTrigged() {
        return !intakeProximitySensor.get();
    }

    public void stopIntake(){
        intakeLeader.stopMotor();
    }

    public void stopShooter(){
        shooterLeader.stopMotor();
    }

    public void stopAll(){
        intakeLeader.stopMotor();
        shooterLeader.stopMotor();
    }  
}
