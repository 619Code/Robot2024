package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.DriveToPointCommand;

public class RobotContainer {

    //////////////////////////////////////////////////////////////////////////////////////

    //                         SUBSYSTEM ENABLE/DISABLE CONTROLS                        //

    //////////////////////////////////////////////////////////////////////////////////////

    private final boolean enableDrivetrain  = true;
    private final boolean enableHinge       = false;
    private final boolean enableManipulator = false;
    private final boolean enableClimb       = false;

    //////////////////////////////////////////////////////////////////////////////////////

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final XboxController driverOne = new XboxController(0);

    public RobotContainer() {

        if (enableDrivetrain) {
            swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        }

        if (enableHinge) {
            //
        }

        if (enableManipulator) {
            //
        }

        if (enableClimb) {
            //
        }

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        if (enableDrivetrain) {
            //
        }

        if (enableHinge) {
            //
        }

        if (enableManipulator) {
            //
        }

        if (enableClimb) {
            //
        }
        
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public Command getAutonomousCommand() {

        return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
        .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
        .andThen( () -> swerveSubsystem.resetOdometry())
        .andThen(new DriveToPointCommand(swerveSubsystem, -1, 2, 0.05))
        ;
        
    }
}