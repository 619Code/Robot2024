package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.helpers.Crashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder absoluteEncoder;

    private final PIDController turningPidController;

    private  DCMotorSim driveMotorSim;
    private  DCMotorSim turningMotorSim;

    private final String ModuleName;

    class SwerveModuleTuning {
        private final DoubleEntry driveMomentSub;
        private final DoubleEntry turningMomentSub;
        private final DoubleEntry turningkPSub;
        private final DoubleEntry turningkISub;
        private final DoubleEntry turningkDSub;

        SwerveModuleTuning(
            double driveMoment,
            double turningMoment,
            double turningkP,
            double turningkI,
            double turningkD,
            String moduleName
        ) {
            NetworkTable table = NetworkTableInstance
                                    .getDefault()
                                    .getTable("SwerveModuleTuning - " + moduleName);
            
            driveMomentSub = table.getDoubleTopic("DriveMoment").getEntry(driveMoment);
            turningMomentSub = table.getDoubleTopic("TurningMoment").getEntry(turningMoment);
            turningkPSub = table.getDoubleTopic("TurningMomementkP").getEntry(turningkP);
            turningkISub = table.getDoubleTopic("TurningMomementkI").getEntry(turningkI);
            turningkDSub = table.getDoubleTopic("TurningMomementkD").getEntry(turningkD);
            
            // Publish once so they show up in the table
            driveMomentSub.set(driveMoment);
            turningMomentSub.set(turningMoment);
            turningkPSub.set(turningkP);
            turningkISub.set(turningkI);
            turningkDSub.set(turningkD);
        }

        double getDriveMoment() {
            return driveMomentSub.get();
        }

        double getTurningMoment() {
            return turningMomentSub.get();
        }

        double getTurningkP() {
            return turningkPSub.get();
        }

        double getTurningkI() {
            return turningkISub.get();
        }

        double getTurningkD() {
            return turningkDSub.get();
        }
    }

    private final SwerveModuleTuning moduleTuning;

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

        /* Tuning */
        moduleTuning = new SwerveModuleTuning(
            0.000418,
            0.000418, 
            0.1,
            0.0,
            0.0,
            this.ModuleName
        );

        turningPidController = new PIDController(
            moduleTuning.getTurningkP(), 
            moduleTuning.getTurningkI(), 
            moduleTuning.getTurningkD()
        );
        turningPidController.enableContinuousInput(-180, 180);

        /* ----------------- Simulation ---------------- */

        // TODO: These numbers are bogus. Fix them!
        driveMotorSim = new DCMotorSim(
            DCMotor.getNEO(1), 
            1.0, // This is a n:m where n>m represents a reduction 
            moduleTuning.getDriveMoment() // This is the moment of inertia of the motor. This number is made up
        );
        turningMotorSim = new DCMotorSim(
            DCMotor.getNEO(1), 
            1.0 / ModuleConstants.kTurningMotorGearRatio, // This is a n:m where n>m represents a reduction 
            moduleTuning.getTurningMoment() // This is the moment of inertia of the motor. This number is made up
        );
        
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
        if (Robot.isReal()) {
            return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
        } else {
            double speed = driveMotorSim.getAngularPositionRotations() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
            Rotation2d angle = Rotation2d.fromDegrees(getAbsoluteEncoderDeg());
            return new SwerveModulePosition(
                speed,
                angle
            );
        }
    }

    // What the heck are the units on this thing?!?
    public double getDriveVelocity() {
        if (Robot.isReal()) {
            return driveEncoder.getVelocity();
        } else {
            return driveMotorSim.getAngularVelocityRPM() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
        }
    }

    public double getAbsoluteEncoderDeg() {
        if (Robot.isReal()) {
            return (absoluteEncoder.getAbsolutePosition().getValue() * 360);// - absoluteEncoderOffset;
        } else {
            return turningMotorSim.getAngularPositionRad() * 180 / Math.PI;
        }

        //should be in degrees???
        // No. This should absolutely be in radians. I won't change it because I don't know the 
        // codebase very well yet, but we almost always convert back this value to radians
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        System.out.println(driveMotorSim.getAngularPositionRad());
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // Update the parameters 
        turningPidController.setPID(
            moduleTuning.getTurningkP(), 
            moduleTuning.getTurningkI(), 
            moduleTuning.getTurningkD()
        );
        

        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        // This is the jank line that we thought broke things earlier.
        //state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromDegrees(getAbsoluteEncoderDeg())).getCos();
        
        // Calculate the drive output from the drive PID controller. ;}
        double driveSpeed = MathUtil.clamp(state.speedMetersPerSecond  / Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond/* / 360*/, -.80 ,.80);
  
        driveMotor.set(driveSpeed);
        driveMotorSim.setInputVoltage(driveSpeed * RobotController.getBatteryVoltage());

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
        turningMotorSim.setInputVoltage(turnSpeed * 12.0);

        //System.out.println(ModuleName + "- DriveMotorCommand: " + driveSpeed + " - True Angle: " + getAbsoluteEncoderRad() + " AngleSetPoint: " + state.angle.getDegrees() + " AngleMotorCommand: " + turnSpeed);
    }

    public void updateSim(double dt) {
        driveMotorSim.update(dt);
        turningMotorSim.update(dt);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);

        driveMotorSim.setInputVoltage(0.0);
        turningMotorSim.setInputVoltage(0.0);
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