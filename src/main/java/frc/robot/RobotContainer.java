package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TestHingeCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestHingeSubsystem;
import frc.robot.commands.DriveToPointCommand;

public class RobotContainer {

    //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final TestHingeSubsystem testHingeSubsystem = new TestHingeSubsystem();
    private final CommandXboxController driverOne = new CommandXboxController(0);

    public RobotContainer() {
        //swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        
        testHingeSubsystem.setDefaultCommand(new TestHingeCommand(testHingeSubsystem, driverOne));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //
        
    }

    public SwerveSubsystem getSwerve() {
        return null;//swerveSubsystem;
    }

    public Command getAutonomousCommand() {

    //     return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
    //     .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
    //         new Rotation2d(0), 
    //         new Rotation2d(0),
    //         new Rotation2d(0),
    //         new Rotation2d(0)}))
    //     .andThen( () -> swerveSubsystem.resetOdometry())
    //     .andThen(new DriveToPointCommand(swerveSubsystem, -1, 2, 0.05))
    //     ;
    return null;    
    }
}