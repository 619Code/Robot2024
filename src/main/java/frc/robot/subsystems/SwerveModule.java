package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.helpers.Crashboard;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder; //BAD CRINGE
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    //private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private final PIDController drivePidController; 


    private final CANcoder absoluteEncoder;

    private final String ModuleName;

    public SwerveModule(String ModuleName, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, SensorDirectionValue positiveDirection) {

        this.ModuleName = ModuleName;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = positiveDirection;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        //absoluteEncoder.configAllSettings(canCoderConfiguration);
        absoluteEncoder.getConfigurator().apply(canCoderConfiguration);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        configureMotor(driveMotor, driveMotorReversed);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        configureMotor(turningMotor, turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        //turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Deg);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2DegPerSec);
 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-180, 180);

        this.drivePidController = new PIDController(ModuleConstants.kPModuleDriveController, 0, ModuleConstants.kDModuleDriveController);

        resetEncoders();
    }

    private void configureMotor(CANSparkMax motor, Boolean inverted) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake); // change to IdleMode.kBrake when ready.
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(39);
        motor.burnFlash();
    }
/////Get position from encoders rather than gyro
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderDeg() {
        return (absoluteEncoder.getAbsolutePosition().getValue() * 360);// - absoluteEncoderOffset;

        //should be in degrees???
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public void setDesiredState(SwerveModuleState state) {
        
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        // This is the jank line that we thought broke things earlier.
        state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromDegrees(getAbsoluteEncoderDeg())).getCos();
        
        // Calculate the drive output from the drive PID controller. ;}
        double driveSpeed = MathUtil.clamp(state.speedMetersPerSecond  / Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond/* / 360*/, -.80 ,.80);
  
        driveMotor.set(driveSpeed);
        // Crashboard.toDashboard("driveSpeed", driveSpeed, "Swerve");
        // Crashboard.toDashboard("speed in m/s", state.speedMetersPerSecond, "Swerve");
        
                
        double turnSpeed = (turningPidController.calculate(getAbsoluteEncoderDeg(), state.angle.getDegrees()));
        //System.out.println("Turn Speed Calculated " + this.ModuleName + ": " + turnSpeed);
        if (turnSpeed > 0)
            turnSpeed = Math.min(turnSpeed, .2);
        else
            turnSpeed = Math.max(turnSpeed, -.2);
        
        //System.out.println("Turn Speed Final " + this.ModuleName + ": " + turnSpeed);
        // Crashboard.toDashboard(ModuleName + "Turn Speed Final", turnSpeed, "Swerve");

        //

        turningMotor.set(turnSpeed);
        //System.out.println(ModuleName + "- DriveMotorCommand: " + driveSpeed + " - True Angle: " + getAbsoluteEncoderRad() + " AngleSetPoint: " + state.angle.getDegrees() + " AngleMotorCommand: " + turnSpeed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public CANcoder getCANcoder() {
        return this.absoluteEncoder;
    }

    public void logIt() {
      Crashboard.toDashboard(ModuleName + " Wheel Angle", this.getAbsoluteEncoderDeg(), "swerve");
      Crashboard.toDashboard(ModuleName + " Abs. Enc.", this.absoluteEncoder.getAbsolutePosition().getValueAsDouble(), "swerve");
      //Crashboard.toDashboard(ModuleName  + " Encoder Value Drive", this.driveMotor.getEncoder().getPosition(), "swerve");
      Crashboard.toDashboard(ModuleName  + " Deg. Turn", this.turningMotor.getEncoder().getPosition(), "swerve");
    }
}