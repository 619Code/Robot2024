package frc.robot.commands.AutoCommands;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectoryCommand extends Command {
    private final ChoreoTrajectory trajectory;
    private final Timer timer;
    private final SwerveSubsystem swerveSubsystem;
    private final boolean isRedAlliance;
    private boolean isFinished = false;
    private final DoubleArrayPublisher commandedPosePublisher;
    
    public FollowTrajectoryCommand(String trajectoryName, SwerveSubsystem swerveSubsystem, boolean isRedAlliance) {
        
        ChoreoTrajectory loadedTrajectory;
        try { 
            loadedTrajectory = Choreo.getTrajectory(trajectoryName);
        } catch (Exception e) {
            loadedTrajectory = new ChoreoTrajectory();
        }

        this.trajectory = loadedTrajectory;
        
        this.timer = new Timer();
        this.swerveSubsystem = swerveSubsystem;
        this.isRedAlliance = isRedAlliance;

        // This prevents other commands from using the swerve subsytem while we're using it
        addRequirements(swerveSubsystem); 

        commandedPosePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("CommandedTrajectory").publish();
        Pose2d initialPose = this.trajectory.getInitialPose();
        commandedPosePublisher.set(new double[]{initialPose.getX(), initialPose.getY(), initialPose.getRotation().getRadians()});
    }

    @Override
    public void initialize() {
        timer.start();
        isFinished = false;
    }


    @Override
    public void execute() {
        /* The idea is to grab a point in the trajectory and interpret it
         * It should give use our current pose which we can then calcluate a wheel state
         * via ChassisSpeeds. 
         */
        double time = timer.get();
        try {
            ChoreoTrajectoryState currentState = trajectory.sample(time, isRedAlliance);
            ChassisSpeeds speed = currentState.getChassisSpeeds();
            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speed));
            
            commandedPosePublisher.set(new double[]{currentState.x, currentState.y, currentState.heading});
            isFinished = currentState == trajectory.getFinalState();
        } catch (Exception e) {
            isFinished = true;
            e.printStackTrace(System.err);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // This may compare two different objects ...
        return isFinished;
    }
}
