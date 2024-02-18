package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class HingeSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax hingeLeader;
    private final CANSparkMax hingeFollower;

    private final CANcoder encoder;

    private final ArmFeedforward ff;

    public HingeSubsystem() {
        super(
            new ProfiledPIDController(
                Constants.HingeConstants.kHingeP, 
                Constants.HingeConstants.kHingeI, 
                Constants.HingeConstants.kHingeD, 
                new TrapezoidProfile.Constraints(
                    Constants.HingeConstants.kHingeMaxVelocityRadPerSecond, 
                    Constants.HingeConstants.KHingeMaxAccelerationRadPerSecond)),
        0);
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

        ff = new ArmFeedforward(Constants.HingeConstants.kHingeS, Constants.HingeConstants.kHingeG, Constants.HingeConstants.kHingeV, Constants.HingeConstants.kHingeA);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);

        hingeLeader.setVoltage(output + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return getAbsolteAngle();
    }

    @Override
    public void periodic() {
        super.periodic();
        checkLimits();
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

    public boolean isAtPosition(double setpoint, double deadzone) {
        if (getAbsolteAngle() <= (setpoint + deadzone) || getAbsolteAngle() >= (setpoint - deadzone)) {
            return true;
        }
        else {
            return false;
        }
    }
}
