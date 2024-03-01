package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.ManipulatorSubsystem;
=======
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TestHingeCommand;
import frc.robot.subsystems.HingeSubsystem;
>>>>>>> main
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestHingeSubsystem;
import frc.robot.commands.DriveToPointCommand;
<<<<<<< HEAD
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopManipulatorCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Can I do this?

public class RobotContainer {

 //   private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

    private final CommandXboxController driverOne = new CommandXboxController(0);
    private final CommandXboxController operatorOne = new CommandXboxController(1);

    public RobotContainer() {
 //       swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
=======
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

        

    //////////////////////////////////////////////////////////////////////////////////////

    //                         SUBSYSTEM ENABLE/DISABLE CONTROLS                        //

    //////////////////////////////////////////////////////////////////////////////////////

    private final boolean enableDrivetrain  = true;
    private final boolean enableHinge       = false;
    private final boolean enableManipulator = false;
    private final boolean enableClimb       = false;

    //////////////////////////////////////////////////////////////////////////////////////

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer() {

        if (enableDrivetrain) {
            swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        }

        if (enableHinge) {
            hingeSubsystem.setDefaultCommand(new GoToShootPosCommand(hingeSubsystem));
        }

        if (enableManipulator) {
            //
        }

        if (enableClimb) {
            //
        }

>>>>>>> main
        configureButtonBindings();
    }

    private void configureButtonBindings() {
<<<<<<< HEAD

        //  =========   MANIPULATOR BINDINGS   =========

        //  Run intake command on {y} press
        Trigger intakeTrigger = operatorOne.y();
        intakeTrigger.onTrue(new IntakeCommand(manipulatorSubsystem));
        // Stop manipulator {y} lift
        intakeTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

        //  Run shoot speaker command on {a} press
        Trigger shootSpeakerTrigger = operatorOne.a();
        shootSpeakerTrigger.onTrue(new ShootCommand(manipulatorSubsystem, Constants.ManipulatorConstants.outtakeSpeedSpeaker, Constants.ManipulatorConstants.intakeSpeedWhenOuttaking, Constants.ManipulatorConstants.speakerShooterVelocityToReachBeforeFeedingNote));
        // Stop manipulator on {a} lift
        shootSpeakerTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

        //  Run shoot amp command on {b} press
        Trigger shootAmpTrigger = operatorOne.b();
        shootAmpTrigger.onTrue(new ShootCommand(manipulatorSubsystem, Constants.ManipulatorConstants.outtakeSpeedAmp, Constants.ManipulatorConstants.intakeSpeedWhenOuttaking, Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote));
        // Stop manipulator on {a} lift
        shootAmpTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));


        //   =========   OTHER BINDINGS   =========
    
    }

    // public SwerveSubsystem getSwerve() {
    //     return swerveSubsystem;
    // }
=======
        
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
>>>>>>> main

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
<<<<<<< HEAD

        return null;
        
=======
    return null;    
>>>>>>> main
    }
}