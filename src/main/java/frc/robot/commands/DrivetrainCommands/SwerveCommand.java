package frc.robot.commands.DrivetrainCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

    private final double slow = 0.4;

    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier getDx;
    private final DoubleSupplier getDy; 
    private final DoubleSupplier getDOmega;
    private final BooleanSupplier activateSlowMode;
    private final BooleanSupplier reorient;

    private SlewRateLimiter driveLimiterX, driveLimiterY, turnLimiter;

    public SwerveCommand(SwerveSubsystem swerveSubsystem, Joystick controller) {
        this(
            swerveSubsystem, 
            () -> {return controller.getRawAxis(1);},
            () -> {return controller.getRawAxis(0);},
            () -> {return controller.getRawAxis(3);},
            () -> {return controller.getRawButton(1);},
            () -> {return controller.getRawButtonPressed(2);}
        );
    }

    public SwerveCommand(
        SwerveSubsystem swerveSubsystem, 
        DoubleSupplier dx, 
        DoubleSupplier dy, 
        DoubleSupplier domega,
        BooleanSupplier activateSlowMode,
        BooleanSupplier reorient
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.getDx = dx;
        this.getDy = dy;
        this.getDOmega = domega;
        this.activateSlowMode = activateSlowMode;
        this.reorient = reorient;

        addRequirements(swerveSubsystem);

        driveLimiterX = new SlewRateLimiter(16);
        driveLimiterY = new SlewRateLimiter(16);
        turnLimiter = new SlewRateLimiter(20);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // reorient button!

        if (reorient.getAsBoolean()) {
            swerveSubsystem.reorientMidMatch();
        }

        // end reorient

        boolean slowMode = activateSlowMode.getAsBoolean();
        
        //double xSpeed = Math.abs(controller.getLeftY()) > OIConstants.kDeadband ? controller.getLeftY() : 0.0;
        double xSpeed = getDx.getAsDouble();
        if (Math.abs(xSpeed) <= Constants.OIConstants.kDeadband) {
            xSpeed = 0;
        }

        xSpeed = driveLimiterX.calculate(xSpeed);

        xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        if (slowMode) xSpeed *= slow;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        double ySpeed = getDy.getAsDouble();
        if (Math.abs(ySpeed) <= Constants.OIConstants.kDeadband) {
            ySpeed = 0;
        }
        
        ySpeed = driveLimiterY.calculate(ySpeed);
        
        ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;   

        if (slowMode) ySpeed *= slow;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        double turningSpeed = getDOmega.getAsDouble();
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