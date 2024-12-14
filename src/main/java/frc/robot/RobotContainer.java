package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import java.util.Optional;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.NewLimelight;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.LedAnimationCommand;
import frc.robot.commands.AutoCommands.AutoShootCommand;
import frc.robot.commands.AutoCommands.LimelightCenterOnAprilTag;
import frc.robot.commands.AutoCommands.NewLimelightAutoCommand;
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
import frc.robot.helpers.Crashboard;
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
    private final AutoSwitchBoardSub switchBoard = new AutoSwitchBoardSub(false);
    private final ledSubsystem LEDs = new ledSubsystem();      
    
    private final NewLimelight limelight = new NewLimelight();

    //////////////////////////////////////////////////////////////////////////////////////

    //                         SUBSYSTEM ENABLE/DISABLE CONTROLS                        //

    //////////////////////////////////////////////////////////////////////////////////////

    public static final boolean enableDrivetrain      = true;
    public static final boolean enableHinge           = true;
    public static final boolean enableManipulator     = true;
    public static final boolean enableClimb           = false;
    public static final boolean enableGroundIntake    = false;
    public static final boolean enableAutoSwitchBoard = true;
    public static final boolean enableLEDs            = true;

    public static final boolean enableLimelight      = true; 

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

        if (enableAutoSwitchBoard) {
            //
        }

        if (enableLEDs){
            LEDs.setDefaultCommand(new LedAnimationCommand(LEDs));
        }

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        if (enableDrivetrain) {
            swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
            Trigger triggerTrigger = new JoystickButton(driverOne, 1);
            triggerTrigger.whileTrue(new NewLimelightAutoCommand(swerveSubsystem, limelight));
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


        return new LimelightCenterOnAprilTag(swerveSubsystem, limelight);


        //  return Commands.repeatingSequence(
        //     new WaitCommand(1).
        //     andThen(new DriveToPointCommand(swerveSubsystem, limelight.GetGoofyAhhHeading(), 0.04))
        //  );

        // return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(new WaitCommand(1))
        //         .andThen(new DriveToPointCommand(swerveSubsystem, limelight.GetGoofyAhhHeading(), 0.04));

        // return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
        //         .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
        //     new Rotation2d(0), 
        //     new Rotation2d(0),
        //     new Rotation2d(0),
        //     new Rotation2d(0)}))
        //         .andThen(new WaitCommand(0.2))
        //         .andThen(new DriveToPointCommand(swerveSubsystem, limelight.GetGoofyAhhHeading(), 0.1));



        /* 
        int selectedAuto = 0;
        if (enableAutoSwitchBoard) selectedAuto = switchBoard.getSwitchCombo();
        System.out.println(selectedAuto);

        // ally multiplier is used to set the proper direction for 
        //  red vs blue since they are mirrored
        int allyMultiplier = 1;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            allyMultiplier = ally.get() == Alliance.Blue ? 1 : -1;
        }

        Crashboard.toDashboard("Alliance: ", ally.get().toString(), "Competition");

        if (!switchBoard.shouldTaxi()) {
            //just shoot speaker
            return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(new AutoShootCommand(manipulatorSubsystem));
        } else if(switchBoard.isPositionForward()) {
                //  Shoot, delay and move at end of autonomous            
                 return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(new AutoShootCommand(manipulatorSubsystem))
                .andThen(new WaitCommand(9.0))
                .andThen(new DriveToPointCommand(swerveSubsystem, -1.5,0, 0.3));
                

        }else if(switchBoard.isPositionSourceSide()){

            //   Starting source side (automatically detects red or blue)
            return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(() -> swerveSubsystem.resetOdometry())
                .andThen(new AutoShootCommand(manipulatorSubsystem))
                .andThen(new WaitCommand(5.0))
                .andThen(new DriveToPointCommand(swerveSubsystem, -1.7, 1.0 * allyMultiplier, 0.3))
                .andThen(new DriveToPointCommand(swerveSubsystem, -1.7, -1.7 * allyMultiplier, 0.3))
                .andThen(new DriveToPointCommand(swerveSubsystem, -30 * allyMultiplier, 0.15));
                //.andThen(() -> swerveSubsystem.reorientMidMatch());

        } else if (switchBoard.isPositionAmpSide()) {
            //shoot and taxi
            return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(new AutoShootCommand(manipulatorSubsystem))
                .andThen(new WaitCommand(8.0))
                .andThen(new DriveToPointCommand(swerveSubsystem, -0.7, -.3 * allyMultiplier, 0.3))
                .andThen(new DriveToPointCommand(swerveSubsystem, -1.3, 2.1 * allyMultiplier, 0.3))
                .andThen(new DriveToPointCommand(swerveSubsystem, 30 * allyMultiplier, 0.15));
                //.andThen(() -> swerveSubsystem.reorientMidMatch());
        }

        else return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
            new Rotation2d(0), 
            new Rotation2d(0),
            new Rotation2d(0),
            new Rotation2d(0)}))
                .andThen(new AutoShootCommand(manipulatorSubsystem));
                */
    }
}