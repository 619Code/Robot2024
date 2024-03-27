package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.AutoCommands.AutoShootCommand;
import frc.robot.commands.ClimbCommands.ClimbCommand;
import frc.robot.commands.ClimbCommands.ClimbCommandDown;
import frc.robot.commands.ClimbCommands.ClimbCommandUp;
import frc.robot.commands.DrivetrainCommands.DriveToPointCommand;
import frc.robot.commands.DrivetrainCommands.SwerveCommand;
import frc.robot.commands.HingeCommands.HingeInitializeCommand;
import frc.robot.commands.ShooterCommands.ClimbWithArmCommand;
import frc.robot.commands.ShooterCommands.DefaultShootCommand;
import frc.robot.commands.ShooterCommands.ClimbWithArmCommandDown;
import frc.robot.commands.ShooterCommands.GoToAmpPosCommand;
import frc.robot.commands.ShooterCommands.GoToInakePosCommand;
import frc.robot.commands.ShooterCommands.GoToInakePosCommandGroundIntakeTesting;
import frc.robot.commands.ShooterCommands.GoToShootPosCommand;
import frc.robot.commands.ShooterCommands.GoToTrussPosCommand;
import frc.robot.commands.ShooterCommands.IntakeCommand;
import frc.robot.commands.ShooterCommands.OuttakeCommand;
import frc.robot.commands.ShooterCommands.ShootCommand;
import frc.robot.commands.ShooterCommands.StopManipulatorCommand;
import frc.robot.commands.Unused.TestHingeCommand;
import frc.robot.helpers.AutoSelector;
import frc.robot.subsystems.AutoSwitchBoardSub;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ledSubsystem;

public class RobotContainer {

    //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //private final TestHingeSubsystem testHingeSubsystem = new TestHingeSubsystem();
    private final HingeSubsystem hingeSubsystem = new HingeSubsystem();
    private final Joystick driverOne = new Joystick(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
    private final AutoSwitchBoardSub switchBoard = new AutoSwitchBoardSub(true);
    private final ledSubsystem LEDs = new ledSubsystem();

        

    //////////////////////////////////////////////////////////////////////////////////////

    //                         SUBSYSTEM ENABLE/DISABLE CONTROLS                        //

    //////////////////////////////////////////////////////////////////////////////////////

    public static final boolean enableDrivetrain      = true;
    public static final boolean enableHinge           = true;
    public static final boolean enableManipulator     = true;
    public static final boolean enableClimb           = true;
    public static final boolean enableGroundIntake    = false;
    public static final boolean enableAutoSwitchBoard = true;
    public static final boolean enableLEDs            = false;

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
            manipulatorSubsystem.setDefaultCommand(new DefaultShootCommand(manipulatorSubsystem));
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

            //operatorController.rightTrigger().whileTrue(new GoToInakePosCommand(hingeSubsystem));

            operatorController.x().whileTrue(new GoToTrussPosCommand(hingeSubsystem));
            
        }

        if (enableManipulator) {
            //  =========   MANIPULATOR BINDINGS   =========
            Trigger shooTrigger = operatorController.leftTrigger();
            shooTrigger.onTrue(new ShootCommand(manipulatorSubsystem));
            shooTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

            Trigger intakeTrigger = operatorController.y();
            intakeTrigger.onTrue(new IntakeCommand(manipulatorSubsystem));
            intakeTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

            Trigger outtakeTrigger = operatorController.rightTrigger();
            outtakeTrigger.whileTrue(new OuttakeCommand(manipulatorSubsystem));

        }

        // if (enableClimb && false) {
        //     Trigger newClimbTrigger = operatorController.back();

        //      newClimbTrigger.onTrue(new ClimbCommand(climbSubsystem));
        //      newClimbTrigger.onFalse(new ClimbCommand(climbSubsystem));
            
        //     if (enableHinge) {
        //         newClimbTrigger.whileTrue(new ClimbWithArmCommand(hingeSubsystem));
        //     }
        // }

        if (enableClimb)
        {
            Trigger upClimbTrigger = operatorController.back();
            Trigger downClimbTrigger = operatorController.start();

            upClimbTrigger.onTrue(new ClimbCommandUp(climbSubsystem));
            upClimbTrigger.onTrue(new ClimbWithArmCommand(hingeSubsystem));

            downClimbTrigger.onTrue(new ClimbCommandDown(climbSubsystem));
            downClimbTrigger.onTrue(new SequentialCommandGroup(new WaitCommand(.75), new ClimbWithArmCommandDown(hingeSubsystem)));
        }

        if (enableGroundIntake) {
            // Trigger intakeTrigger = operatorController.rightTrigger();
            // intakeTrigger.whileTrue(new GroundIntakeCommand(groundIntakeSubsystem));
        }

        if (enableLEDs) {
            LEDs.setColor(0, 0, 255);
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

        // return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(new AutoShootCommand(manipulatorSubsystem));

        int selectedAuto = 0;
        if (enableAutoSwitchBoard) selectedAuto = switchBoard.getSwitchCombo();
        System.out.println(selectedAuto);

        if (selectedAuto == 1) {
            //just shoot speaker
            return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(new AutoShootCommand(manipulatorSubsystem));
        } else {
            //shoot and taxi
            return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(new AutoShootCommand(manipulatorSubsystem))
                .andThen(new DriveToPointCommand(swerveSubsystem, -3.0,0, 0.1))
                .andThen(new DriveToPointCommand(swerveSubsystem, 0, 1, 0.1))
                //.andThen(new DriveToPointCommand(swerveSubsystem, 45, 0.1))
                ;
        }
    }

        // switch (selectedAuto) {
        //     // No auto.
        //     case 0 -> {
        //         return null;
        //     }
        //     // Basic taxi.
        //     case 1 -> {
        //         return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(new DriveToPointCommand(swerveSubsystem, -1.5, 0, 0.1))
        //         ;
        //     }
        //     // Just shoot speaker.
        //     case 2 -> {
        //         return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(new AutoShootCommand(manipulatorSubsystem))
        //         ;
        //     }
        //     // Just shoot amp.
        //     case 3 -> {
        //         return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(
        //             Commands.parallel(
        //                 new GoToAmpPosCommand(hingeSubsystem),
        //                 new AutoShootCommand(manipulatorSubsystem)
        //             )
        //         )
        //         ;
        //     }
        //     // Speaker shot, then taxi. ONLY WORKS DEAD ON.
        //     case 4 -> {
        //         return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(new AutoShootCommand(manipulatorSubsystem))
        //         .andThen(new DriveToPointCommand(swerveSubsystem, -1.5, 0, 0.1))
        //         ;
        //     }
        //     // Amp shot, then taxi.
        //     case 5 -> {
        //         return null;
        //     }
        //     // nothing yet
        //     case 6 -> {
        //         return null;
        //     }
        //     // option 7, revert to 0 just in case.
        //     default -> {
        //         return null;
        //     }
        //}

        // EMERGENCY AUTO! SHOOT ONLY! -------------------------------------------------------------------------------------------
        
        // return Commands.runOnce( () -> swerveSubsystem.zeroHeading())
        // .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        // .andThen( () -> swerveSubsystem.resetOdometry())
        // .andThen(new AutoShootCommand(manipulatorSubsystem))
        // ;

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