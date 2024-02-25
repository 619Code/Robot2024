package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.Crashboard;

public class TestHingeSubsystem extends SubsystemBase {
    private final CANSparkMax hingeLeader;
    private CANSparkMax hingeFollower;

    private DutyCycleEncoder encoder;

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
        //hingeFollower.getEncoder().setInverted(true);

        // encoder = new CANcoder(Constants.HingeConstants.kAbsoluteEncoderPort);

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
    
    public double getAbsoluteAngle() {
        return (encoder.getAbsolutePosition());
    }

    public double getAbsoluteDegrees() {
        return ((.87 - encoder.getAbsolutePosition())/.26) * (72) + (60);
    }
}
