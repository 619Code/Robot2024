package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.Crashboard;

public class TestHingeSubsystem extends SubsystemBase {
    private final CANSparkMax hingeLeader;
    private CANSparkMax hingeFollower;

    private DutyCycleEncoder encoder;
    private RelativeEncoder hingeLeaderRelativeEncoder;     
    private RelativeEncoder hingeFollowerRelativeEncoder;

    public TestHingeSubsystem() {
        //left motor
        hingeLeader = new CANSparkMax(Constants.HingeConstants.kHingeLeaderPort, MotorType.kBrushless);
        hingeLeader.restoreFactoryDefaults();
        hingeLeader.setIdleMode(IdleMode.kBrake);
        hingeLeader.setSmartCurrentLimit(35);
        hingeLeader.setInverted(Constants.HingeConstants.kHingeLeaderInverted);
        // hingeLeader.getEncoder().setInverted(true);

        // right motor
        hingeFollower = new CANSparkMax(Constants.HingeConstants.kHingeFollowerPort, MotorType.kBrushless);
        hingeFollower.restoreFactoryDefaults();
        hingeFollower.setIdleMode(IdleMode.kCoast);
        hingeFollower.setSmartCurrentLimit(35);
        hingeFollower.setInverted(Constants.HingeConstants.kHingeFollowerInverted);

        hingeLeaderRelativeEncoder = hingeLeader.getEncoder();
        hingeFollowerRelativeEncoder = hingeFollower.getEncoder();
    
        //hingeFollower.follow(hingeLeader);
        encoder = new DutyCycleEncoder(Constants.HingeConstants.kAbsoluteEncoderPort);
        encoder.setDistancePerRotation(360.0);
        encoder.setPositionOffset(Constants.HingeConstants.kAbsoluteEncoderOffset);
    }



    @Override 
    public void periodic() {
        //
        Crashboard.toDashboard("Left Motor Encoder", hingeLeader.getEncoder().getPosition(), "Hinge");
        Crashboard.toDashboard("Right Motor Encoder", hingeLeader.getEncoder().getPosition(), "Hinge");
        Crashboard.toDashboard("AbsoluteEncoderPositon", getAbsoluteAngle(), "Hinge");
        Crashboard.toDashboard("AbsoluteEncoderAngle", getAbsoluteDegrees(), "Hinge");
    }

    public void spinge(double speed) {
        hingeLeader.set(speed);
        hingeFollower.set(speed);
    }

    public void stopHinge(){
        hingeLeader.stopMotor();
        hingeFollower.stopMotor();
    }
    

    public void resetRelativeEncoders() {
        hingeLeaderRelativeEncoder.setPosition(0);
        hingeFollowerRelativeEncoder.setPosition(0);
    }
    
    public double getAbsoluteAngle() {
        return (encoder.getAbsolutePosition());
    }
    public double getAbsoluteDegrees() {
        //return ((.87 - encoder.getAbsolutePosition())/.26) * (72) + (60);
        return parseRawAbsEncoderValue(encoder.getAbsolutePosition(), 
            Constants.HingeConstants.rawEncoderLow, 
            Constants.HingeConstants.rawEncoderHigh, 
            Constants.HingeConstants.degreesLow, 
            Constants.HingeConstants.degreesHigh);
    }
    public double parseRawAbsEncoderValue(double rawAbsoluteEncoderValue, double rawEncoderLow, double rawEncoderHigh, double degreesLow, double degreesHigh) {
       
        return (rawAbsoluteEncoderValue-rawEncoderLow)/(rawEncoderHigh-rawEncoderLow)*(degreesHigh-degreesLow)+degreesLow;

    }

    public void SetRelativeEncoderSoftLimits(float lowerLimit, float upperLimit){

        hingeLeader.setSoftLimit(SoftLimitDirection.kForward, upperLimit);
        hingeFollower.setSoftLimit(SoftLimitDirection.kForward, upperLimit);
        
        hingeLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
        hingeFollower.enableSoftLimit(SoftLimitDirection.kForward, true);


        hingeLeader.setSoftLimit(SoftLimitDirection.kReverse, lowerLimit);
        hingeFollower.setSoftLimit(SoftLimitDirection.kReverse, lowerLimit);

        hingeLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
        hingeFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);

    }
}
