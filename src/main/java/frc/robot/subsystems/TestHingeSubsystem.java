package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestHingeSubsystem extends SubsystemBase {
    private final CANSparkMax hingeLeader;
    private final CANSparkMax hingeFollower;

    private final CANcoder encoder;

    public TestHingeSubsystem() {
        hingeLeader = new CANSparkMax(Constants.HingeConstants.kHingeLeaderPort, MotorType.kBrushless);
        hingeLeader.restoreFactoryDefaults();
        hingeLeader.setIdleMode(IdleMode.kBrake);
        hingeLeader.setSmartCurrentLimit(35);
        hingeLeader.setInverted(Constants.HingeConstants.kHingeLeaderInverted);

        hingeFollower = new CANSparkMax(Constants.HingeConstants.kHingeFollowerPort, MotorType.kBrushless);
        hingeFollower.restoreFactoryDefaults();
        hingeFollower.setIdleMode(IdleMode.kBrake);
        hingeFollower.setSmartCurrentLimit(35);
        hingeFollower.setInverted(Constants.HingeConstants.kHingeFollowerInverted);

        encoder = new CANcoder(Constants.HingeConstants.kAbsoluteEncoderPort);

        hingeFollower.follow(hingeLeader);
    }

    public void spinge(double speed) {
        hingeLeader.set(speed);
    }

    public void stopHinge(){
        hingeLeader.stopMotor();
        hingeFollower.stopMotor();
    }
    
}
