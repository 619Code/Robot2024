package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DrivetrainCommands.DriveToPointCommand;
import frc.robot.helpers.Crashboard;
import frc.robot.helpers.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.SwerveSubsystem;


public class NewLimeLightCommand extends Command {

    private final SwerveSubsystem swerveSubsystem; 
    private final DoubleSubscriber txSub;
    private final PIDController PIDCon = new PIDController(0, 0, 0);
    private final DoubleEntry kP;
    private final DoubleEntry kI;
    private final DoubleEntry kD;

    public NewLimeLightCommand(SwerveSubsystem swerve) {

        swerveSubsystem = swerve;
        txSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tx").subscribe(0);
        kP = NetworkTableInstance.getDefault().getDoubleTopic("kP").getEntry(0);
        kI = NetworkTableInstance.getDefault().getDoubleTopic("kI").getEntry(0);
        kD = NetworkTableInstance.getDefault().getDoubleTopic("kD").getEntry(0);

        kP.set(0);
        kI.set(0);
        kD.set(0);

    }

    public Measure<Angle> getLimelightHeadingDegrees() {
        
        //NetworkTableEntry tx = table.getEntry(LimelightHelpers.getTX(""));
        return Units.Degree.of(txSub.get());

    }
    @Override
    public void initialize() {
        
}

    @Override
    public void execute() {
        double turningSpeedRadiansPerSecond = PIDCon.calculate(getLimelightHeadingDegrees().magnitude(), 0.0);

        Rotation2d currentHeading = Rotation2d.fromDegrees(-swerveSubsystem.getHeading()); //inverted
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeedRadiansPerSecond, currentHeading);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveSubsystem.getModuleStates(), Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        getLimelightHeadingDegrees().magnitude() <= 0.5;
    }

}
