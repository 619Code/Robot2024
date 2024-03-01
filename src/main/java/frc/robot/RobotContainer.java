package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TestHingeCommand;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestHingeSubsystem;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.GoToAmpPosCommand;
import frc.robot.commands.GoToInakePosCommand;
import frc.robot.commands.GoToShootPosCommand;
import frc.robot.commands.HingeInitializeCommand;

public class RobotContainer {

    //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //private final TestHingeSubsystem testHingeSubsystem = new TestHingeSubsystem();
    private final HingeSubsystem hingeSubsystem = new HingeSubsystem();
    private final CommandXboxController driverOne = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public RobotContainer() {
        //swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        
       // new HingeInitializeCommand(hingeSubsystem).execute();

    //    Commands.runOnce(() -> new HingeInitializeCommand(hingeSubsystem));
        //.andThen(() -> hingeSubsystem.setDefaultCommand(new GoToShootPosCommand(hingeSubsystem)));

        // Commands.runOnce(new HingeInitializeCommand(hingeSubsystem)).andThen(() -> {
        //    hingeSubsystem.setDefaultCommand(new GoToShootPosCommand(hingeSubsystem)); 
        // });

        hingeSubsystem.setDefaultCommand(new GoToShootPosCommand(hingeSubsystem));

        //hingeSubsystem.setDefaultCommand(new GoToShootPosCommand(hingeSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        
        operatorController.a().whileTrue(new GoToInakePosCommand(hingeSubsystem));
        operatorController.y().whileTrue(new GoToAmpPosCommand(hingeSubsystem));

        operatorController.start().onTrue(new HingeInitializeCommand(hingeSubsystem));
        operatorController.b().whileTrue(new TestHingeCommand(hingeSubsystem, operatorController));
    }

    public SwerveSubsystem getSwerve() {
        return null;//swerveSubsystem;
    }

    public void InitializeHinge() {
        new HingeInitializeCommand(hingeSubsystem).schedule();
    }

    public Command getAutonomousCommand() {

        // return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
        // .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        // .andThen( () -> swerveSubsystem.resetOdometry())
        // .andThen(new DriveToPointCommand(swerveSubsystem, -1, 2, 0.05))
        // ;
    return null;    
    }
}