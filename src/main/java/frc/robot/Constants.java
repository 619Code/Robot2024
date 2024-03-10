package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class HingeConstants {
        public static final int kHingeLeaderPort = 52; // left motor
        public static final int kHingeFollowerPort = 61; // right motor
        public static final boolean kHingeLeaderInverted = false;
        public static final boolean kHingeFollowerInverted = true;

        public static final int kAbsoluteEncoderPort = 0;
        public static final int kAbsoluteEncoderOffset = 0;
        public static final double kMaxAngle = 128; //TEMP VALUE
        public static final double kMinAngle = 63; //TEMP VALUE

        public static final float topEncoderSoftLimit = 73;
        public static final float bottomEncoderSoftLimit = -0.01f;
        
        public static final double kHingeP = 0.27; //UNTUNED
        public static final double kHingeI = 0; //UNTUNED
        public static final double kHingeD = 0; //UNTUNED
        public static final double kHingeG = .14; 
        public static final double kHingeV = 5.35; 
        public static final double kHingeA = 0.0; 
        public static final double kHingeS = 0.0;

        public static final double kHingeMaxVelocityRadPerSecond = 0;
        public static final double KHingeMaxAccelerationRadPerSecond = 0;

        public static final double kIntakeAngle = 110.4;
        public static final double kShootingAngle = Constants.HingeConstants.kMinAngle;
        public static final double kAmpAngle = 127.5;

        public static final double rawEncoderLow = .88;
        public static final double rawEncoderHigh = .617;
        public static final double degreesLow = 60;
        public static final double degreesHigh = 128;

        }

  public static final int CANdleid = 20;
    public static final class ModuleConstants {
        // these constants should be correct for the current 4ki module we are using 
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 1/21.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Deg = kTurningMotorGearRatio * 360;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2DegPerSec = kTurningEncoderRot2Deg / 60;


        // We may need to tune this for the PID turning
        
        // we may need to tune it so this is not set in stone
        public static final double kPTurning = .01;
        public static final double kDTurning = 0;

        public static final double kPModuleDriveController = .01;
        public static final double kDModuleDriveController = 0;
    }

    public static final class DriveConstants {

        //Used for uh, idk. 
        public static final double kNavxUnitsToMetersConversion = 19.4;

        // we need to update this // no longer needs to be updated: I measured from center of axle to center of axle
        //thse seem to be based off of the base dimensions
        public static final double kTrackWidth = Units.inchesToMeters(21); 
        public static final double kWheelBase = Units.inchesToMeters(21);

        //This should be relative to the center, but still check documentation about how the grid is set up for swerve kinetics
            //Ive changed it. It looked wrong and I fixed it based on the coordinate system at https://hhs-team670.github.io/MustangLib/frc/team670/robot/utils/math/Translation2d.html
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //backright

        //We need to update these motors
        // frontLeft Module
        // offsets should be drawn from no offest vaule on x tuner
        public static final int kFrontLeftDriveMotorPort = 51; //motors updated
        public static final int kFrontLeftTurningMotorPort = 50; //motors updated
        public static final boolean kFrontLeftDriveMotorReversed = true; //re updated //updated
        public static final boolean kFrontLeftTurningMotorReversed = true; //updated 
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 33; //updated
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false; //updated
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = -.230225; //updated // they want this to be the negative of the reported values?
        public static final SensorDirectionValue kFrontLeftTurningForwardDirection = SensorDirectionValue.CounterClockwise_Positive;

        //we need to updats these motors
        // frontRight Module
        // offset -0.266357
        public static final int kFrontRightDriveMotorPort = 58; //motors updated
        public static final int kFrontRightTurningMotorPort = 57; //motors updated
        public static final boolean kFrontRightDriveMotorReversed = false; //updated
        public static final boolean kFrontRightTurningMotorReversed = true; //updated
        public static final int kFrontRightDriveAbsoluteEncoderPort = 30; //updated 
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false; //updated
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = -0.261475; //updated, in degrees
        public static final SensorDirectionValue kFrontRightTurningForwardDirection = SensorDirectionValue.CounterClockwise_Positive;

        //we need to update these motors
        // backLeft Module
        // offset -0.480713
        public static final int kBackLeftDriveMotorPort = 55; //motors updated
        public static final int kBackLeftTurningMotorPort = 56; //motors updated
        public static final boolean kBackLeftDriveMotorReversed = true; //updated
        public static final boolean kBackLeftTurningMotorReversed = true; //updated
        public static final int kBackLeftDriveAbsoluteEncoderPort = 32; //updated
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false; //updated
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = -0.394287; //updated, in degrees
        public static final SensorDirectionValue kBackLeftTurningForwardDirection = SensorDirectionValue.CounterClockwise_Positive;

        //we need to update these motors
        // backRight Module
        //0.480957
        public static final int kBackRightDriveMotorPort = 62; //motors updated
        public static final int kBackRightTurningMotorPort = 49; //motors updated
        public static final boolean kBackRightDriveMotorReversed = false; //updated
        public static final boolean kBackRightTurningMotorReversed = true; //updated
        public static final int kBackRightDriveAbsoluteEncoderPort = 31; //updated
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; //updated
        public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 0.397461; //updated, in degrees
        public static final SensorDirectionValue kBackRightTurningForwardDirection = SensorDirectionValue.CounterClockwise_Positive;

        
        //NOTE: these are not used in actual code they are just used to define max based on physical contraints
        //If you want to ignore these then change the limit on the max speeds manually 
        //these seem to be mostly fine but we may need change some things
        // also we need to change the physical dimensions of our base if we are going to use this 
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5676.0 / 60.0 * ModuleConstants.kDriveEncoderRot2Meter; //4.47332629073 m/s
        public static final double kPhysicalMaxAngularSpeedDegreesPerSecond = 360; //2 * Math.PI; //kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0 * 3);

        //These are the variables that determine the max speeds of our swerve drive
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * (0.9); // Divide by 4 for slow testing. 
        public static final double kTeleDriveMaxAngularSpeedDegreesPerSecond = kPhysicalMaxAngularSpeedDegreesPerSecond * (0.75);
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    //these we can ignore for now since they are only for autonmous 
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedDegreesPerSecond = DriveConstants.kPhysicalMaxAngularSpeedDegreesPerSecond / 10;
        
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationDegreesPerSecondSquared = 45.0;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedDegreesPerSecond,
                        kMaxAngularAccelerationDegreesPerSecondSquared);
    }

    //this is just to set a deadzone so we don't need to move the stick
    public static final class OIConstants {
        public static final double kDeadband = 0.05;
    }

    public static final class PIDConstants {
        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoldSecondsSquaredPerMeter = 0;

        public static final double kPDriveVel = 0;
        public static final double kD = 0;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ManipulatorConstants {
        public static final int kIntakeLeaderPort = 60;  //20;  Test bed  value
        public static final boolean kInakeLeaderInverted = false;
        public static final int kShooterLeaderPort = 54; //10; TEst bed value
        public static final boolean kShooterLeaderInverted = true;

        public static final double intakeSpeed = 0.3;
        public static final double intakeSpeedWhenOuttaking = 0.6;
        public static final double outtakeSpeedSpeaker = 0.7;
        public static final double outtakeSpeedAmp = 0.7;

        public static final int kIntakeSensorPort = 9;  //0f test bed value

        public static final int speakerShooterVelocityToReachBeforeFeedingNote = 3000;
        public static final int ampShooterVelocityToReachBeforeFeedingNote = 1500;
        
    }

    public static final class ClimbConstants {
        public static final int kLeftArmForwardPort = 5;
        public static final int kLeftArmBackwardPort = 4;
        
        public static final int kRightArmForwardPort = 3;
        public static final int kRightArmBackwardPort = 2;
    }
} 