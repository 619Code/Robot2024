 package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPointCommand extends Command {

    private SwerveSubsystem swerve;
    private Pose2d firstPos, secondPos;
    private Transform2d dPos;
    private double baseMaxSpeed, rotationMaxSpeed; 
    private double calculatedXSpeed, calculatedYSpeed, calculatedRotSpeed; 
    private double dx, dy, dTheta;
    private boolean stickyX, stickyY, stickyR;

    public DriveToPointCommand(SwerveSubsystem subsystem, double changeinXMeters, double changeinYMeters, double percentageOfMaxSpeed) {
        stickyX = false;
        stickyY = false;
        stickyR = false;

        swerve = subsystem;
        dPos = new Transform2d(
            new Translation2d(
                changeinXMeters * Constants.DriveConstants.kNavxUnitsToMetersConversion, changeinYMeters * Constants.DriveConstants.kNavxUnitsToMetersConversion
            ),
            Rotation2d.fromDegrees(0)
        );

        baseMaxSpeed = Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * percentageOfMaxSpeed;
        rotationMaxSpeed = Constants.DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond * percentageOfMaxSpeed * percentageOfMaxSpeed;

        addRequirements(subsystem);
    }

    public DriveToPointCommand(SwerveSubsystem subsystem, double changeInHeadingDegrees, double percentageOfMaxSpeed) {
        stickyX = false;
        stickyY = false;
        stickyR = false;

        swerve = subsystem;
        dPos = new Transform2d(
            new Translation2d(0, 0), 
            Rotation2d.fromDegrees(changeInHeadingDegrees)
        );
        
        baseMaxSpeed = Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * percentageOfMaxSpeed;
        rotationMaxSpeed = Constants.DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond * percentageOfMaxSpeed * percentageOfMaxSpeed;

        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {
        
        firstPos = swerve.getPose2d();

        secondPos = firstPos.transformBy(dPos);

        System.out.println("Starting Odometry Position: (X/Y/R), X: " + swerve.getPose2d().getX() + ", Y: " + swerve.getPose2d().getY() + ", R: " + swerve.getPose2d().getRotation().getDegrees());


        // Get the time it would take for each item to reach the destination if it is at the max speed

        dx = (secondPos.getX() - firstPos.getX());
        dy = (secondPos.getY() - firstPos.getY());
        dTheta = (secondPos.getRotation().getDegrees() - firstPos.getRotation().getDegrees());

        if (Math.abs(dTheta) < 3) { dTheta = 0;}
        if (Math.abs(dx) < 0.1) { dx = 0;}
        if (Math.abs(dy) < 0.1) { dy = 0;}

        // The times it would take for each item of interest to reach it's destination if it moves at it's max speed

        double xSecMax, ySecMax, tSecMax;

        xSecMax = dx / baseMaxSpeed;
        ySecMax = dy / baseMaxSpeed;
        tSecMax = dTheta / rotationMaxSpeed;        

        //  Get the longest of these times. In other words, the operation should be performed as fast as the slowest movement can go.

        double bottleneckTime = Math.max(Math.max(Math.abs(xSecMax), Math.abs(ySecMax)), Math.abs(tSecMax));

        System.out.println("XSM: " + xSecMax + ", YSM: " + ySecMax + ", TSM: " + tSecMax);
        System.out.println("BOTTLENECK: " + bottleneckTime);

        calculatedXSpeed = dx / bottleneckTime; // literally meters / seconds
        calculatedYSpeed = dy / bottleneckTime; 
        calculatedRotSpeed = dTheta / bottleneckTime;

        // Right now, the calculated X and Y speeds are in meters/second, and not scalar values between 0 and 1.

        Rotation2d currentHeading = Rotation2d.fromDegrees(-swerve.getHeading()); //inverted

        System.out.println("DX: " + dx + ", DY: " + dy + ", DTHETA: " + dTheta);
        System.out.println("CXS: " + calculatedXSpeed + ", CYS: " + calculatedYSpeed + ", CRS: " + calculatedRotSpeed);

        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(calculatedXSpeed, calculatedYSpeed, calculatedRotSpeed, currentHeading); //from Field
        swerve.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));

    }

    @Override
    public void execute() {
        //if (isFinished()) {end(true);}
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(calculatedXSpeed, calculatedYSpeed, calculatedRotSpeed, Rotation2d.fromDegrees(-swerve.getHeading())); //from Field
        swerve.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    @Override
    public boolean isFinished() {

        //  System.out.println("X !!" + (swerve.getPose2d().getX() - secondPos.getX()) + "!!");
        //  System.out.println("Y !!" + (swerve.getPose2d().getY() - secondPos.getY()) + "!!");
        //  System.out.println("R !!" + (swerve.getPose2d().getRotation().getDegrees() - secondPos.getRotation().getDegrees()) + "!!");

         boolean xArrived = Math.abs(swerve.getPose2d().getX() - secondPos.getX()) <= 0.4;
         boolean yArrived = Math.abs(swerve.getPose2d().getY() - secondPos.getY()) <= 0.4;
         boolean tArrived = Math.abs(swerve.getPose2d().getRotation().getDegrees() - secondPos.getRotation().getDegrees()) <= 4;

         if (xArrived) {
            stickyX = true;
            calculatedXSpeed = 0;
         }
         if (yArrived) {
            stickyY = true;
            calculatedYSpeed = 0;
         }
         if (tArrived) {
            stickyR = true;
            calculatedRotSpeed = 0;
         }

         return stickyX && stickyY && stickyR;

        // System.out.println("X !!" + (swerve.getPose2d().getX() - secondPos.getX()) + "!!");
        // return (Math.abs(secondPos.getX() - swerve.getPose2d().getX()) <= 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending Odometry Position: (X/Y/R), X: " + swerve.getPose2d().getX() + ", Y: " + swerve.getPose2d().getY() + ", R: " + swerve.getPose2d().getRotation().getDegrees());
        swerve.stopModules();
    }
}