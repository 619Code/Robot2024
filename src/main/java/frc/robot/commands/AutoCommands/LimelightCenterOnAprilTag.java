package frc.robot.commands.AutoCommands;

import java.io.Console;

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
import edu.wpi.first.wpilibj.Timer;
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

    private DoubleEntry kpRot2NetworkEntry;
    private DoubleEntry kiRot2NetworkEntry;
    private DoubleEntry kdRot2NetworkEntry;

    private DoubleEntry kpMovementNetworkEntry;
    private DoubleEntry kiMovementNetworkEntry;
    private DoubleEntry kdMovementNetworkEntry;

    

    private NetworkTableEntry targetPoseTable;

  //  IntegerEntry filterTapsNetworkEntry;
    int filterTaps = 3;

 //   private final PIDController rotationPid;
    private final PIDController YmovementPid;
    private final PIDController rot2Pid;


//    private final DoublePublisher anglePub;
    private final DoublePublisher desiredSpeedPub;

    private LinearFilter rotationFilter;
    private LinearFilter xMovementFilter;
    private LinearFilter rot2Filter;

    double[] targetPosData;


    Timer generalTimer;


    private enum CenteringStates{
        INITIAL_ORIENTATION,
        MOVING_TOWARDS_POINT
    }

    private CenteringStates currentState;


    public LimelightCenterOnAprilTag(SwerveSubsystem _swerveSubsystem, NewLimelight _limelightSub){
        limelightSubsystem = _limelightSub;
        swerveSubsystem = _swerveSubsystem;

   //     rotationPid = new PIDController(0.07, 0, 0.0016);
        YmovementPid = new PIDController(2, 0, 0.1);
        rot2Pid = new PIDController(0.07, 0, 0.0016);

  //      anglePub = NetworkTableInstance.getDefault().getDoubleTopic("LimelightAngle").publish();
        desiredSpeedPub = NetworkTableInstance.getDefault().getDoubleTopic("LimelightDesiredSpeed").publish();

        kpRotationNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kpR").getEntry(0.0);
        kiRotationNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kiR").getEntry(0.0);
        kdRotationNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kdR").getEntry(0.0);

        kpRot2NetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kpR2").getEntry(0.0);
        kiRot2NetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kiR2").getEntry(0.0);
        kdRot2NetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kdR2").getEntry(0.0);

        kpMovementNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kpM").getEntry(0.0);
        kiMovementNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kiM").getEntry(0.0);
        kdMovementNetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kdM").getEntry(0.0);

        targetPoseTable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace");
   //     filterTapsNetworkEntry = NetworkTableInstance.getDefault().getIntegerTopic("fTaps").getEntry(0);

        kpRotationNetworkEntry.set(0.0);
        kiRotationNetworkEntry.set(0.0);
        kdRotationNetworkEntry.set(0.0);

        kpRot2NetworkEntry.set(0.0);
        kiRot2NetworkEntry.set(0.0);
        kdRot2NetworkEntry.set(0.0);

        kpMovementNetworkEntry.set(0.0);
        kiMovementNetworkEntry.set(0.0);
        kdMovementNetworkEntry.set(0.0);

   //     filterTapsNetworkEntry.set(filterTaps);

        rotationFilter = LinearFilter.movingAverage(filterTaps);
        xMovementFilter = LinearFilter.movingAverage(3);
        rot2Filter = LinearFilter.movingAverage(3);

        generalTimer = new Timer();

        addRequirements(_limelightSub);
        addRequirements(_swerveSubsystem);
    }


    @Override
    public void initialize() {
        SwitchToState(CenteringStates.INITIAL_ORIENTATION);
    }

    @Override
    public void execute() {
      //  limelightSubsystem.SendRandomDataToShuffleboard();
        
      RetrieveDataFromTables();

      switch(currentState) {
        case INITIAL_ORIENTATION:
        //  AimTowardsAprilTag();
          AimPerpindicularToAprilTag();

          CheckStateMachineInitialOrientationStage();
          break;
        case MOVING_TOWARDS_POINT:
          MovingToPoint();

          CheckStateMachineMovingTopositionStage();
          break;
        default:
            break;
      }

    }


    private void CheckStateMachineInitialOrientationStage(){

      boolean isInRange = Math.abs(targetPosData[4]) < 4;

      if(isInRange){
        if(generalTimer.get() > 0.75){
          SwitchToState(CenteringStates.MOVING_TOWARDS_POINT);
        }
      }else{
        generalTimer.reset();
      }
    }

    private void CheckStateMachineMovingTopositionStage(){

      boolean isInRange = Math.abs(targetPosData[0]) < 0.009;

      if(isInRange){
        if(generalTimer.get() > 0.75){
          SwitchToState(CenteringStates.INITIAL_ORIENTATION);
        }
      }else{
        generalTimer.reset();
      }
    }


    private void RetrieveDataFromTables(){
      targetPosData = targetPoseTable.getDoubleArray(new double[6]);
    }


    private void SwitchToState(CenteringStates _state){
      currentState = _state;
      generalTimer.reset();
      generalTimer.start();
    }

    private void MovingToPoint(){

        double targetXOffset = targetPosData[0];
        targetXOffset = xMovementFilter.calculate(targetXOffset);

   //     double targetYOffset = data[1];
    //    double targetZOffset = data[2];

   //     Crashboard.toDashboard("Tz: ", targetZOffset, "Limelight");

        Crashboard.toDashboard("Tx: ", targetPosData[0], "Limelight");
        Crashboard.toDashboard("Ty: ", targetPosData[1], "Limelight");
        Crashboard.toDashboard("Tz: ", targetPosData[2], "Limelight");
        Crashboard.toDashboard("Pitch: ", targetPosData[3], "Limelight");
        Crashboard.toDashboard("Yaw: ", targetPosData[4], "Limelight");
        Crashboard.toDashboard("Roll: ", targetPosData[5], "Limelight");


    //    YmovementPid.setPID(kpMovementNetworkEntry.get(), kiMovementNetworkEntry.get(), kdMovementNetworkEntry.get());

        
        double ySpeed = -YmovementPid.calculate(targetXOffset);

        ySpeed = Math.min(2, ySpeed);

       
        //  The speeds are negative because the robot is programmed to have the front be the back
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, -ySpeed, 0, Rotation2d.fromDegrees(-swerveSubsystem.getHeading())); //from Field
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }
    
  private void AimPerpindicularToAprilTag(){
        // int newFilterValue = (int)filterTapsNetworkEntry.get();
        // if(newFilterValue != filterTaps){
        //      filterTaps = newFilterValue; 
        //      filter = LinearFilter.movingAverage(filterTaps);
        // }

        double angleDifference = targetPosData[4];
        angleDifference = rot2Filter.calculate(angleDifference);

        //  Disabling because advantage scope keeps setting my stuff to 0
   //     rot2Pid.setPID(kpRot2NetworkEntry.get(), kiRot2NetworkEntry.get(), kdRot2NetworkEntry.get());


        Crashboard.toDashboard("Perp angle difference: ", angleDifference, "Limelight");

       double speedRadiansPerSecond = -rot2Pid.calculate(angleDifference, 0);

       desiredSpeedPub.set(speedRadiansPerSecond);
       
       ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, speedRadiansPerSecond, Rotation2d.fromDegrees(-swerveSubsystem.getHeading())); //from Field
       swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    //  Aims directly to the april tag
   /*  private void AimTowardsAprilTag(){
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
    }*/

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
