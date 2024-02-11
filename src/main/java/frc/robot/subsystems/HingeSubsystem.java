package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;

public class HingeSubsystem extends SubsystemBase {
    private final CANSparkMax hingeLeader;
    private final CANSparkMax hingeFollower;

    private final CANcoder encoder;

    private final PIDController pid;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double ff;

    public HingeSubsystem() {
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

        kP = Constants.HingeConstants.kHingeP;
        kI = Constants.HingeConstants.kHingeI;
        kD = Constants.HingeConstants.kHingeD;
        ff = Constants.HingeConstants.kHingeF;
        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void periodic() {
    }

    public void checkLimits() {
        if (getAbsolteAngle() > Constants.HingeConstants.kMaxAngle || getAbsolteAngle() < Constants.HingeConstants.kMinAngle) {
            stop();
        }
    }

    public void stop() {
        hingeLeader.stopMotor();
    }

    public double getAbsolteAngle() { // -180, 360
        return (encoder.getAbsolutePosition().getValue() * 360);
    }

    public double getAbsoluteRotations() { // -.5 , 1
        return (encoder.getAbsolutePosition().getValue());
    }

    public void goToPosition(double setpoint) {
        hingeLeader.set(pid.calculate(getAbsoluteRotations(), setpoint));
    }
}
