package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.NewLimelight;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightCenterOnAprilTag extends Command {
    
    private final NewLimelight limelightSubsystem;
    private final SwerveSubsystem swerveSubsystem;
 
    private DoubleEntry kpRotationNetworkEntry;
    private DoubleEntry kiRotationNetworkEntry;
    private DoubleEntry kdRotationNetworkEntry;

    private DoubleEntry kpMovementNetworkEntry;
    private DoubleEntry kiMovementNetworkEntry;
    private DoubleEntry kdMovementNetworkEntry;

    

    private NetworkTableEntry targetPoseTable;

  //  IntegerEntry filterTapsNetworkEntry;
    int filterTaps = 3;

    private final PIDController rotationPid;
    private final PIDController YmovementPid;


    private final DoublePublisher anglePub;
    private final DoublePublisher desiredSpeedPub;

    private LinearFilter rotationFilter;
    private LinearFilter xMovementFilter;


    private enum CenteringStates{
        INITIAL_ORIENTATION,
        MOVING_TOWARDS_POINT
    }

    private CenteringStates currentStage = CenteringStates.MOVING_TOWARDS_POINT;


    public LimelightCenterOnAprilTag(SwerveSubsystem _swerveSubsystem, NewLimelight _limelightSub){
        limelightSubsystem = _limelightSub;
        swerveSubsystem = _swerveSubsystem;

        rotationPid = new PIDController(0.07, 0, 0.0016);
        YmovementPid = new PIDController(0, 0, 0);

        anglePub = NetworkTableInstance.getDefault().getDoubleTopic("LimelightAngle").publish();
        desiredSpeedPub = NetworkTableInstance.getDefault().getDoubleTopic("LimelightDesiredSpeed").publish();

        kpRotationNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kpR").getEntry(0.0);
        kiRotationNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kiR").getEntry(0.0);
        kdRotationNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kdR").getEntry(0.0);

        kpMovementNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kpM").getEntry(0.0);
        kiMovementNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kiM").getEntry(0.0);
        kdMovementNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kdM").getEntry(0.0);

        targetPoseTable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace");
   //     filterTapsNetworkEntry = NetworkTableInstance.getDefault().getIntegerTopic("fTaps").getEntry(0);

        kpRotationNetworkEntry.set(0.0);
        kiRotationNetworkEntry.set(0.0);
        kdRotationNetworkEntry.set(0.0);

        kpMovementNetworkEntry.set(0.0);
        kiMovementNetworkEntry.set(0.0);
        kdMovementNetworkEntry.set(0.0);

   //     filterTapsNetworkEntry.set(filterTaps);

        rotationFilter = LinearFilter.movingAverage(filterTaps);
        xMovementFilter = LinearFilter.movingAverage(3);

        addRequirements(_limelightSub);
        addRequirements(_swerveSubsystem);
    }


    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
      //  limelightSubsystem.SendRandomDataToShuffleboard();
        
      switch(currentStage) {
        case INITIAL_ORIENTATION:
          AimTowardsAprilTag();
          break;
        case MOVING_TOWARDS_POINT:
          MovingToPoint();
          break;
        default:
            break;
      }

    }

    private void MovingToPoint(){


        double[] data = targetPoseTable.getDoubleArray(new double[6]);

        double targetXOffset = data[0];
        targetXOffset = xMovementFilter.calculate(targetXOffset);

   //     double targetYOffset = data[1];
    //    double targetZOffset = data[2];

   //     Crashboard.toDashboard("Tz: ", targetZOffset, "Limelight");


        YmovementPid.setPID(kpMovementNetworkEntry.get(), kiMovementNetworkEntry.get(), kdMovementNetworkEntry.get());


        double xSpeed = 0;
        

        double ySpeed = -YmovementPid.calculate(targetXOffset);

        ySpeed = Math.min(0.5, ySpeed);

       
        //  The speeds are negative because the robot is programmed to have the front be the back
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(-xSpeed, -ySpeed, 0, Rotation2d.fromDegrees(-swerveSubsystem.getHeading())); //from Field
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }
    
    private void AimTowardsAprilTag(){
        // int newFilterValue = (int)filterTapsNetworkEntry.get();
        // if(newFilterValue != filterTaps){
        //      filterTaps = newFilterValue; 
        //      filter = LinearFilter.movingAverage(filterTaps);
        // }

        double angleDifference = limelightSubsystem.GetGoofyAhhHeading();
        angleDifference = rotationFilter.calculate(angleDifference);

        rotationPid.setPID(kpRotationNetworkEntry.get(), kiRotationNetworkEntry.get(), kdRotationNetworkEntry.get());


        Crashboard.toDashboard("Angle: ", angleDifference, "Limelight");

        anglePub.set(angleDifference);

        double speedRadiansPerSecond = -rotationPid.calculate(angleDifference, 0);

        desiredSpeedPub.set(speedRadiansPerSecond);
       
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, speedRadiansPerSecond, Rotation2d.fromDegrees(-swerveSubsystem.getHeading())); //from Field
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
