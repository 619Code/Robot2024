package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands.AutoShootCommand;
import frc.robot.commands.ClimbCommands.ClimbCommand;
import frc.robot.commands.DrivetrainCommands.DriveToPointCommand;
import frc.robot.commands.DrivetrainCommands.SwerveCommand;
import frc.robot.commands.HingeCommands.HingeInitializeCommand;
import frc.robot.commands.ShooterCommands.ClimbWithArmCommand;
import frc.robot.commands.ShooterCommands.GoToAmpPosCommand;
import frc.robot.commands.ShooterCommands.GoToInakePosCommand;
import frc.robot.commands.ShooterCommands.GoToShootPosCommand;
import frc.robot.commands.ShooterCommands.IntakeCommand;
import frc.robot.commands.ShooterCommands.ShootCommand;
import frc.robot.commands.ShooterCommands.StopManipulatorCommand;
import frc.robot.commands.Unused.TestHingeCommand;
import frc.robot.helpers.AutoSelector;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //private final TestHingeSubsystem testHingeSubsystem = new TestHingeSubsystem();
    private final HingeSubsystem hingeSubsystem = new HingeSubsystem();
    private final Joystick driverOne = new Joystick(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

        

    //////////////////////////////////////////////////////////////////////////////////////

    //                         SUBSYSTEM ENABLE/DISABLE CONTROLS                        //

    //////////////////////////////////////////////////////////////////////////////////////

    public static final boolean enableDrivetrain  = true;
    public static final boolean enableHinge       = true;
    public static final boolean enableManipulator = true;
    public static final boolean enableClimb       = true;

    //////////////////////////////////////////////////////////////////////////////////////

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer() {

        AutoSelector.setGyro(swerveSubsystem.getGyro());

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

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        if (enableDrivetrain) {
            swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        }

        if (enableHinge) {
            hingeSubsystem.setDefaultCommand(new GoToShootPosCommand(hingeSubsystem));
                
            operatorController.y().whileTrue(new GoToInakePosCommand(hingeSubsystem));
            operatorController.a().whileTrue(new GoToAmpPosCommand(hingeSubsystem));

            //operatorController.start().onTrue(new HingeInitializeCommand(hingeSubsystem));
            operatorController.b().whileTrue(new TestHingeCommand(hingeSubsystem, operatorController));

            Trigger strongClimbTrigger = operatorController.x();
            strongClimbTrigger.whileTrue(new ClimbWithArmCommand(hingeSubsystem));
            
        }

        if (enableManipulator) {
            //  =========   MANIPULATOR BINDINGS   =========
            Trigger shooTrigger = operatorController.leftTrigger();
            shooTrigger.onTrue(new ShootCommand(manipulatorSubsystem));
            shooTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

            Trigger intakeTrigger = operatorController.y();
            intakeTrigger.onTrue(new IntakeCommand(manipulatorSubsystem));
            intakeTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

        }

        if (enableClimb) {
            Trigger climbTrigger = operatorController.back();
            climbTrigger.toggleOnTrue(new ClimbCommand(climbSubsystem));
            //climbTrigger.toggleOnFalse(new ClimbCommand(climbSubsystem));
        }
        
        //   =========   OTHER BINDINGS   =========
    
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
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
        // .andThen(new AutoShootCommand(manipulatorSubsystem))
        // .andThen(Commands.either(
        //     Commands.either(
        //       new DriveToPointCommand(swerveSubsystem, -1, -2, 0.1) // Option 1 (A & B)    // LEFT
        //     , new DriveToPointCommand(swerveSubsystem, -1, 2, 0.1) // Option 2 (!A & B)   //RIGHT
        //     , () -> AutoSelector.isfacingLeft())         // Selector A, 1-2 // LEFT OR RIGHT (LEFT TRUE RIGHT FALSE)
        //     , new DriveToPointCommand(swerveSubsystem, -3, 0, 0.1) // Option 3 (!(A|B) & C) //CENTER
        //     , () -> AutoSelector.isFacingSide()         // Selector B, A-3 // SIDE OR CENTER (SIDE TRUE CENTER FALSE)
        //     ))
        // ;

        // PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK PLEASE WORK 

        /*
         WE ARE GOING TO ASSUME A CERTAIN ORIENTATION FOR THE SPEAKER.
         IT SHOULD LOOK LIKE THIS.


         /                     CENTER
         |                     _____          |
         |              LEFT  /     \ RIGHT   |
         |------------------------------------|
             me ☺

        ALL AUTOS NEED TO:
            - SHOOT
            - TAXI
        CENTER WILL TAXI STRAIGHT.
        LEFT CAN TAXI STRAIGHT BUT SHOULD NOT.
        LEFT WILL TAXI AT A 60 DEGREE ANGLE TO THE DRIVER STATION.
        RIGHT WILL DO THE EXACT INVERSE OF LEFT.
        :3c
        */

        // EMERGENCY AUTO! SHOOT ONLY! -------------------------------------------------------------------------------------------
        
        return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
        .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
        .andThen( () -> swerveSubsystem.resetOdometry())
        .andThen(new AutoShootCommand(manipulatorSubsystem))
        ;

        // EMERGENCY AUTO! DRIVE ONLY! -------------------------------------------------------------------------------------------
        
        // return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
        // .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        // .andThen( () -> swerveSubsystem.resetOdometry())
        // .andThen(new DriveToPointCommand(swerveSubsystem, -3, 0, 0.1))
        // ;
        
    }
}