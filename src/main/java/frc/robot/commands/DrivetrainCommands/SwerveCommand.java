package frc.robot.commands.DrivetrainCommands;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

    private final double slow = 0.3;

    private final SwerveSubsystem swerveSubsystem;
    private final Joystick controller;
    private CANcoder FrontLeftCoder;
    private CANSparkMax FrontLeftTurnSpark;

    private SlewRateLimiter driveLimiter, driveLimiterX, driveLimiterY, turnLimiter;

    public SwerveCommand(SwerveSubsystem swerveSubsystem, Joystick controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        addRequirements(swerveSubsystem);
        this.FrontLeftCoder = swerveSubsystem.frontLeft.getCANcoder();
        this.FrontLeftTurnSpark = swerveSubsystem.frontLeft.turningMotor;

        driveLimiterX = new SlewRateLimiter(16);
        driveLimiterY = new SlewRateLimiter(16);
        turnLimiter = new SlewRateLimiter(20);
        //driveLimiter = new SlewRateLimiter(0.5);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // reorient button!

        if (controller.getRawButtonPressed(2)) {
            swerveSubsystem.reorientMidMatch();
        }

        // end reorient

        boolean slowMode = controller.getRawButton(1);

        // :3

        //
        //System.out.println("ANGLE = " + swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() + ", SPEED = " + FrontLeftTurnSpark.get());

        // \:3
        
        //double xSpeed = Math.abs(controller.getLeftY()) > OIConstants.kDeadband ? controller.getLeftY() : 0.0;
        double xSpeed = controller.getRawAxis(1);
        if (Math.abs(xSpeed) <= Constants.OIConstants.kDeadband) {
            xSpeed = 0;
        }

        xSpeed = driveLimiterX.calculate(xSpeed);

        xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        if (slowMode) xSpeed *= slow;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        double ySpeed = controller.getRawAxis(0);
        if (Math.abs(ySpeed) <= Constants.OIConstants.kDeadband) {
            ySpeed = 0;
        }
        
        ySpeed = driveLimiterY.calculate(ySpeed);
        
        ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;   

        if (slowMode) ySpeed *= slow;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        double turningSpeed = controller.getRawAxis(3);
        if (Math.abs(turningSpeed) <= Constants.OIConstants.kDeadband) {
            turningSpeed = 0;
        }

        turningSpeed = turnLimiter.calculate(turningSpeed);

        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond;

        if (slowMode) turningSpeed *= slow;

        Crashboard.toDashboard("kTeleDriveMaxAngularSpeedDegreesPerSecond", DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond, "navx");
        //turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        //turningSpeed = 0;
        //System.out.println("xSpeed: " + xSpeed + " ySpeed: " + ySpeed + " turningSpeed: " + turningSpeed);

        double turningSpeedRadiansPerSecond = Rotation2d.fromDegrees(turningSpeed).getRadians();
        Rotation2d currentHeading = Rotation2d.fromDegrees(-swerveSubsystem.getHeading()); //inverted
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeedRadiansPerSecond, currentHeading);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveSubsystem.getModuleStates(), Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        Crashboard.toDashboard("turningSpeedRadiansPerSecond", turningSpeedRadiansPerSecond, "navx");
        Crashboard.toDashboard("currentHeading", currentHeading.getRadians(), "navx");
       
        //Crashboard.toDashboard("desired states", DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[0].toString(), "navx");
    }


    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}