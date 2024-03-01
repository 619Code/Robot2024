package frc.robot.commands;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController controller;
    private CANcoder FrontLeftCoder;
    private CANSparkMax FrontLeftTurnSpark;

    private SlewRateLimiter driveLimiter, driveLimiterX, driveLimiterY, turnLimiter;

<<<<<<< HEAD
    public SwerveCommand(SwerveSubsystem swerveSubsystem, CommandXboxController controller) {
=======
    public SwerveCommand(SwerveSubsystem swerveSubsystem, CommandXboxController driverOne) {
>>>>>>> main
        this.swerveSubsystem = swerveSubsystem;
        this.controller = driverOne;
        addRequirements(swerveSubsystem);
        this.FrontLeftCoder = swerveSubsystem.frontLeft.getCANcoder();
        this.FrontLeftTurnSpark = swerveSubsystem.frontLeft.turningMotor;

        driveLimiterX = new SlewRateLimiter(2);
        driveLimiterY = new SlewRateLimiter(2);
        turnLimiter = new SlewRateLimiter(4);
        //driveLimiter = new SlewRateLimiter(0.5);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // :3

        //
        //System.out.println("ANGLE = " + swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() + ", SPEED = " + FrontLeftTurnSpark.get());

        // \:3
        
        double xSpeed = Math.abs(controller.getLeftY()) > OIConstants.kDeadband ? controller.getLeftY() : 0.0;

        xSpeed = driveLimiterX.calculate(xSpeed);

        xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        double ySpeed = Math.abs(controller.getLeftX()) > OIConstants.kDeadband ? controller.getLeftX() : 0.0; 
        
        ySpeed = driveLimiterY.calculate(ySpeed);
        
        ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;        

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        double turningSpeed = Math.abs(controller.getRightX()) > OIConstants.kDeadband ? controller.getRightX() : 0.0; 

        turningSpeed = turnLimiter.calculate(turningSpeed);

        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond;



        Crashboard.toDashboard("kTeleDriveMaxAngularSpeedDegreesPerSecond", DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond, "navx");
        //turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        //turningSpeed = 0;
        //System.out.println("xSpeed: " + xSpeed + " ySpeed: " + ySpeed + " turningSpeed: " + turningSpeed);

        double turningSpeedRadiansPerSecond = Rotation2d.fromDegrees(turningSpeed).getRadians();
        Rotation2d currentHeading = Rotation2d.fromDegrees(-swerveSubsystem.getHeading()); //inverted
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeedRadiansPerSecond, currentHeading);
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