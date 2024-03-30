package frc.robot.commands.DrivetrainCommands;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

    private final double slow = 0.4;

    private final SwerveSubsystem swerveSubsystem;
    private final Joystick controller;
    private CANcoder FrontLeftCoder;
    private CANSparkMax FrontLeftTurnSpark;

    private SlewRateLimiter driveLimiter, driveLimiterX, driveLimiterY, turnLimiter;

    private boolean isAutomaticallyRotatingToAngle = false, hasReachedAngleToAutomaticallyRotateTo = false;
    private double degreeAngleToAutomaticallyFace = 0.0f, amountOfTimeTillGiveUpAutoRotate = 2.0f, degreeAtStartOfAutoRotate = 0, satisfiedAngleDistanceFromDesiredAngle = 5;
    private Timer elapsedTimeSinceInitiatingAutoRotate = new Timer();
    int previousFrameHatInput = -1;

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
        
        double turningSpeed = 0;

        //  Get what hat input was pressed down this frame (-1 if nothing)
        int hatInput = GetHatDown();

        //  Only if we actually got input this frame 
        if (hatInput != -1) {
            //  Point to rotate to:
            //  0: face source
            //  1: face amp
            //  2: face speaker
            initaiteNewRotateToPoint(hatInput);
        }


        if(!isAutomaticallyRotatingToAngle){
            //  This is the regular controller-controlled rotation code

            turningSpeed = controller.getRawAxis(3); // 4 is for thrust buttons. 

            if (Math.abs(turningSpeed) <= Constants.OIConstants.kDeadband) {
                turningSpeed = 0;
            }

            turningSpeed = turnLimiter.calculate(turningSpeed);

            turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond;

            if (slowMode) turningSpeed *= slow;
        }else{
            //  This is the code that automatically rotates to the current angle


            //  If we have reached the angle we were trying to, or we've spent long enough trying...
            if(hasReachedAngleToAutomaticallyRotateTo || elapsedTimeSinceInitiatingAutoRotate.get() >= amountOfTimeTillGiveUpAutoRotate){
                //  stop autorotating and give control back to driver

                isAutomaticallyRotatingToAngle = false;

            }else{
                //  Regular code for automatically rotating

                turningSpeed = degreeAngleToAutomaticallyFace - swerveSubsystem.getHeading(); 
                //   At this point in the code, {turningSpeed} represents the total degree angle between our heading and the desired heading
                //  therefore, if we are close enough, then stop
                if(Math.abs(turningSpeed) <= satisfiedAngleDistanceFromDesiredAngle){
                    //  We are close enough to the desired angle, stop rotating and give control back to driver

                    hasReachedAngleToAutomaticallyRotateTo = true;

                }

                //  How far we are between start and desired angle, represented between 0 and 1
                double interpolationBetweenAngleAtStartOfAutoRotateAndDesiredAngle = (degreeAngleToAutomaticallyFace - swerveSubsystem.getHeading()) / (degreeAngleToAutomaticallyFace - degreeAtStartOfAutoRotate);
                Crashboard.toDashboard("Interpolation value: ", interpolationBetweenAngleAtStartOfAutoRotateAndDesiredAngle, "Swerve");
                
                //  My sketchy ahh PID avoidance
                if(interpolationBetweenAngleAtStartOfAutoRotateAndDesiredAngle < 0.5){
                    turningSpeed = MathUtil.interpolate(0, turningSpeed, interpolationBetweenAngleAtStartOfAutoRotateAndDesiredAngle * 2.0f);  
                }
                if(interpolationBetweenAngleAtStartOfAutoRotateAndDesiredAngle >= 0.5){
                    turningSpeed = MathUtil.interpolate(turningSpeed, 0, (interpolationBetweenAngleAtStartOfAutoRotateAndDesiredAngle - 0.5) * 2.0f);  
                }
                Crashboard.toDashboard("Automatic rotate to angle current speed (unclamped):", turningSpeed, "Swerve");

                //  Clamp to maxSpeed
                turningSpeed = MathUtil.clamp(turningSpeed, -DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond, DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond);
            }
        }

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
    
        previousFrameHatInput = hatInput;
    
    }

    //  Point to rotate to:
    //  0: face source
    //  1: face amp
    //  2: face speaker
    private void initaiteNewRotateToPoint(int pointToRotateTo){

        hasReachedAngleToAutomaticallyRotateTo = false;
        elapsedTimeSinceInitiatingAutoRotate.restart();
        degreeAtStartOfAutoRotate = swerveSubsystem.getHeading();
        
        switch (pointToRotateTo) {
            case 0:
                //  Driver pressed up on hat THIS FRAME
                //  Face source
                degreeAngleToAutomaticallyFace = 70;
                break;
            case 1:
                //  Driver pressed right on hat THIS FRAME
                //  Face amp
                degreeAngleToAutomaticallyFace = -90;
                break;
            case 2:
                //  Driver pressed down on hat THIS FRAME
                //  Face speaker
                degreeAngleToAutomaticallyFace = 180;
                break;
        }
    }


    private int GetHatDown(){

        int output = -1;

        switch(controller.getPOV()){

            case 0:
                output = 0;
                break;
            case 90:
                output = 1;
                break;
            case 180:
                output = 2;
                break;
            case 270:
                output = 3;
                break;
        }

        //  If this input was the same as last frame, then ignore it
        if(previousFrameHatInput == output){

            return -1;

        }

        return output; 

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