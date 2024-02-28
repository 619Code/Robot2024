package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.helpers.Crashboard;

public class HingeSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax hingeLeader;
    private final CANSparkMax hingeFollower;

    private final DutyCycleEncoder encoder;

    private RelativeEncoder hingeLeaderRelativeEncoder;     
    private RelativeEncoder hingeFollowerRelativeEncoder;

    private final ArmFeedforward ff;

    private static double kP;
    private static double kI;
    private static double kD;

    public HingeSubsystem() {
        super(
            new ProfiledPIDController(
                Constants.HingeConstants.kHingeP, 
                Constants.HingeConstants.kHingeI, 
                Constants.HingeConstants.kHingeD, 
                new TrapezoidProfile.Constraints(
                    Constants.HingeConstants.kHingeMaxVelocityRadPerSecond, 
                    Constants.HingeConstants.KHingeMaxAccelerationRadPerSecond)),
        Constants.HingeConstants.kShootingAngle);

        kP = Constants.HingeConstants.kHingeP;
        kI = Constants.HingeConstants.kHingeI;
        kD = Constants.HingeConstants.kHingeD;

        Crashboard.AddSlider("kP", kP, "Hinge", 0, 4);
        Crashboard.AddSlider("kI", kI, "Hinge", 0, 4);
        Crashboard.AddSlider("kD", kD, "Hinge", 0, 4);

        hingeLeader = new CANSparkMax(Constants.HingeConstants.kHingeLeaderPort, MotorType.kBrushless);
        hingeLeader.restoreFactoryDefaults();
        hingeLeader.setIdleMode(IdleMode.kBrake);
        hingeLeader.setSmartCurrentLimit(35);
        hingeLeader.setInverted(Constants.HingeConstants.kHingeLeaderInverted);

        hingeFollower = new CANSparkMax(Constants.HingeConstants.kHingeFollowerPort, MotorType.kBrushless);
        hingeFollower.restoreFactoryDefaults();
        hingeFollower.setIdleMode(IdleMode.kCoast);
        hingeFollower.setSmartCurrentLimit(35);
        hingeFollower.setInverted(Constants.HingeConstants.kHingeFollowerInverted);

        encoder = new DutyCycleEncoder(Constants.HingeConstants.kAbsoluteEncoderPort);
        encoder.setDistancePerRotation(360.0);
        encoder.setPositionOffset(Constants.HingeConstants.kAbsoluteEncoderOffset);

        hingeLeaderRelativeEncoder = hingeLeader.getEncoder();
        hingeFollowerRelativeEncoder = hingeFollower.getEncoder();

        //hingeFollower.follow(hingeLeader);

        ff = new ArmFeedforward(Constants.HingeConstants.kHingeS, Constants.HingeConstants.kHingeG, Constants.HingeConstants.kHingeV, Constants.HingeConstants.kHingeA);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);

        hingeLeader.setVoltage(output + feedforward);
        hingeFollower.setVoltage(output + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return getAbsoluteDegrees();
    }


    @Override
    public void periodic() {
        super.periodic();
        //checkLimits();

        getController().setP(SmartDashboard.getNumber("kP", Constants.HingeConstants.kHingeP));
        getController().setI(SmartDashboard.getNumber("kI", Constants.HingeConstants.kHingeI));
        getController().setD(SmartDashboard.getNumber("kD", Constants.HingeConstants.kHingeD));

        Crashboard.toDashboard("Leader Encoder", hingeLeader.getEncoder().getPosition(), "Hinge");
        Crashboard.toDashboard("Follower Encoder", hingeFollower.getEncoder().getPosition(), "Hinge");
        Crashboard.toDashboard("AbsoluteEncoderPositon", getAbsoluteAngle(), "Hinge");
        Crashboard.toDashboard("AbsoluteEncoderDegrees", getAbsoluteDegrees(), "Hinge");

    }

    // public void checkLimits() {
    //     if (getAbsoluteAngle() > Constants.HingeConstants.kMaxAngle || getAbsoluteAngle() < Constants.HingeConstants.kMinAngle) {
    //         stop();
    //     }
    // }

    public void resetRelativeEncoders() {
        hingeLeaderRelativeEncoder.setPosition(0);
        hingeFollowerRelativeEncoder.setPosition(0);
    }

    public void stop() {
        hingeLeader.stopMotor();
    }

    public double getAbsoluteAngle() {
        return (encoder.getAbsolutePosition());
    }

    public void spinge(double speed) {
        hingeLeader.set(speed);
        hingeFollower.set(speed);
    }

    public void stopHinge(){
        hingeLeader.stopMotor();
        hingeFollower.stopMotor();
    }

    public boolean isAtPosition(double setpoint, double deadzone) {
        if (getAbsoluteDegrees() <= (setpoint + deadzone) || getAbsoluteDegrees() >= (setpoint - deadzone)) {
            return true;
        }
        else {
            return false;
        }
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
