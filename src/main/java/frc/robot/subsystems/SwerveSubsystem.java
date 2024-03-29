package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.AutoSelector;
import frc.robot.helpers.Crashboard;

public class SwerveSubsystem extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;
    public final SwerveModule frontLeft;
    public final SwerveModule frontRight; 
    public final SwerveModule backLeft;
    public final SwerveModule backRight;
    public final StructArrayPublisher<SwerveModuleState> publisher;

    private SwerveDriveKinematics kinematics;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer;

    private final StructArrayPublisher<SwerveModuleState> publisher_current;
    private final StructArrayPublisher<SwerveModuleState> publisher_desired;

    SwerveModuleState[] states = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    double[] statez = new double[] {
        30, 12, 30, 12, 30, 12, 30, 12
    };

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(
            "Front Left",
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kFrontLeftTurningForwardDirection);

        frontRight = new SwerveModule(
            "Front Right",
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kFrontRightTurningForwardDirection);

        backLeft = new SwerveModule(
            "Back Left",
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kBackLeftTurningForwardDirection);

        backRight = new SwerveModule(
            "Back Right",
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg,
            DriveConstants.kBackRightTurningForwardDirection);

        Translation2d frontLeftPos = new Translation2d(0.2667, 0.2667);
        Translation2d frontRightPos = new Translation2d(0.2667, -0.2667);
        Translation2d backLeftPos = new Translation2d(-0.2667, 0.2667);
        Translation2d backRightPos = new Translation2d(-0.2667, -0.2667);

        kinematics = new SwerveDriveKinematics(
            frontLeftPos, frontRightPos, backLeftPos, backRightPos
        );

        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
//MUST USE A / IN THE NAME OR DIE
        publisher_current = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/MyStatesExpected", SwerveModuleState.struct).publish();
        publisher_desired = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/MyStatesDesired", SwerveModuleState.struct).publish();

        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                
            } 
            catch (Exception e) {
            }
        }).start();
    }
    
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
        });
        // Crashboard.toDashboard("Robot Heading", getHeading(), "navx");
        // frontLeft.logIt();
        // frontRight.logIt();
        // backLeft.logIt();
        // backRight.logIt();
        // Crashboard.toDashboard("gyro angle", -gyro.getAngle(), "Odometry");
        // Crashboard.toDashboard("navx odometry pose x", odometer.getPoseMeters().getX(), "Odometry");
        // Crashboard.toDashboard("navx odometry pose y", odometer.getPoseMeters().getY(), "Odometry");

        Crashboard.toDashboard("Robot Heading", getHeading(), "navx");
        frontLeft.logIt();
        frontRight.logIt();
        backLeft.logIt();
        backRight.logIt();
        Crashboard.toDashboard("PRE-MATCH ORIENTATION", (Math.abs(gyro.getAngle()) < 10), "Competition");           // Comp Orientation Check
        Crashboard.toDashboard("DETERMINED POSITION", "" + AutoSelector.getLocation(), "Competition");
        Crashboard.toDashboard("gyro angle", gyro.getAngle(), "navx");
        publisher_current.set(getModuleStates(), 0);
        //System.out.println(getModuleStates()[1].speedMetersPerSecond);
        //System.out.println(getModuleStates()[1].angle);
        //SmartDashboard.putNumber("Front Right Wheel Angle", frontRight.getAbsoluteEncoderDeg());
        //SmartDashboard.putNumber("Back Left Wheel Angle", backLeft.getAbsoluteEncoderDeg());
        //SmartDashboard.putNumber("Back Right Wheel Angle", backRight.getAbsoluteEncoderDeg());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {

        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        publisher.set(desiredStates);
        //System.out.println(desiredStates);
        publisher_desired.set(desiredStates, 0);
        System.out.println(desiredStates);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }
    public Pose2d getPose2d() {
        return odometer.getPoseMeters();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void ResetRelativePositionEncoders()
    {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void resetOdometry() {
        odometer.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, getPose2d());
    }

    public void reorientMidMatch() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } 
            catch (Exception e) {
            }
        }).start();
    }

    public AHRS getGyro() {
        return gyro;
    }
    

    
}