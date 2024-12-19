package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands.AutoShootCommand;
import frc.robot.commands.AutoCommands.FollowTrajectoryCommand;
import frc.robot.commands.ClimbCommands.ClimbCommandDown;
import frc.robot.commands.ClimbCommands.ClimbCommandUp;
import frc.robot.commands.DrivetrainCommands.DriveToPointCommand;
import frc.robot.commands.DrivetrainCommands.SwerveCommand;
import frc.robot.commands.HingeCommands.HingeInitializeCommand;
import frc.robot.commands.ShooterCommands.ClimbWithArmCommand;
import frc.robot.commands.ShooterCommands.ClimbWithArmCommandDown;
import frc.robot.commands.ShooterCommands.GoToAmpPosCommand;
import frc.robot.commands.ShooterCommands.GoToInakePosCommand;
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
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ledSubsystem;

enum Autos {
    DO_NOTHING,
    JUST_SHOOT,
    FORWARD_SIDE,
    SOURCE_SIDE,
    AMP_SIDE
}

public class RobotContainer {

    //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //private final TestHingeSubsystem testHingeSubsystem = new TestHingeSubsystem();
    private final HingeSubsystem hingeSubsystem = new HingeSubsystem();
    private Joystick joystick; 
    private CommandXboxController xBoxController;
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final AutoSwitchBoardSub switchBoard = new AutoSwitchBoardSub(false);
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
    public static final boolean enableLEDs            = true;
    public static final boolean useXboxController     = false;

    //////////////////////////////////////////////////////////////////////////////////////

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer(Autos auto) {

        switch (auto) {
            case JUST_SHOOT:
            {
                //just shoot speaker
                break;
            }
                

            case FORWARD_SIDE:
            {
                // Tell the odeometry where we are
                swerveSubsystem.initializePose(new Pose2d(
                    Units.Meters.of(1.348),
                    Units.Meters.of(5.512),
                    Rotation2d.fromDegrees(180)
                ));
                break;
            }

            case SOURCE_SIDE:
            {
                // Tell the odeometry where we are
                swerveSubsystem.initializePose(new Pose2d(
                    Units.Meters.of(0.672),
                    Units.Meters.of(4.385),
                    Rotation2d.fromDegrees(120)
                ));

                //   Starting source side (automatically detects red or blue)
                break;
            }


            case AMP_SIDE:
            {
                // Tell the odeometry where we are
                swerveSubsystem.initializePose(new Pose2d(
                    Units.Meters.of(0.738),
                    Units.Meters.of(6.661),
                    Rotation2d.fromDegrees(-120)
                ));
                
                //shoot and taxi
                break;
            }
        
            case DO_NOTHING:
            default:
            {
                break;
            }
        }

        AutoSelector.setGyro(swerveSubsystem.getGyro());
    }

    private double axisSmoother(double axis) {
        double deadband = 0.1;
        if (Math.abs(axis) < deadband) {
            return 0.0;
        } 

        // x^2
        return axis * axis * Math.signum(axis);

    }

    private void configureButtonBindings() {

        

        if (enableDrivetrain) {
            if (useXboxController)
            {
                xBoxController = new CommandXboxController(0);
            
                swerveSubsystem.setDefaultCommand(new SwerveCommand(
                    swerveSubsystem, 
                    () -> {return axisSmoother(-xBoxController.getLeftY());}, // dx
                    () -> {return axisSmoother(-xBoxController.getLeftX());}, // dy
                    () -> {return axisSmoother(xBoxController.getRightX());}, //domega
                    (BooleanSupplier)xBoxController.rightBumper(), // slow mode
                    (BooleanSupplier)xBoxController.leftStick() // reorient
                ));
            }
            else
            {
                this.joystick = new Joystick(0);                
                swerveSubsystem.setDefaultCommand(new SwerveCommand(
                    swerveSubsystem, 
                    () -> {
                        return axisSmoother(joystick.getRawAxis(Joystick.kDefaultXChannel));
                    }, // dx
                    () -> {
                        return axisSmoother(joystick.getRawAxis(Joystick.kDefaultYChannel));
                    }, // dy
                    () -> {
                        return axisSmoother(joystick.getRawAxis(Joystick.kDefaultTwistChannel));
                    }, //domega
                    () -> {
                        return joystick.getRawButton(1);
                    }, // slow mode
                    () -> { 
                        return joystick.getRawButton(2);
                    } // reorient
                ));
            }
        }

        if (enableHinge) {

            Trigger intakePosition = useXboxController ? xBoxController.y() : new Trigger(() -> joystick.getRawButton(3));
            Trigger ampPosition = useXboxController ? xBoxController.a() : new Trigger(() -> joystick.getRawButton(4));
            Trigger testHingePosition = useXboxController ? xBoxController.b() : new Trigger(() -> { return joystick.getRawButton(5);});
    
            intakePosition.whileTrue(new GoToInakePosCommand(hingeSubsystem));
            ampPosition.whileTrue(new GoToAmpPosCommand(hingeSubsystem));
            
            DoubleSupplier testHingeSupplier = useXboxController ? () -> {return axisSmoother(xBoxController.getRightY());} : () -> { return joystick.getRawAxis(Joystick.kDefaultYChannel);};

            testHingePosition.whileTrue(new TestHingeCommand(hingeSubsystem, testHingeSupplier));
            
            //trussPosition.whileTrue(new GoToTrussPosCommand(hingeSubsystem));
        }

        if (enableManipulator) {
            //  =========   MANIPULATOR BINDINGS   =========
            Trigger shooTrigger = useXboxController ? xBoxController.leftTrigger() : new Trigger(() -> joystick.getRawButton(8));
            shooTrigger.onTrue(new ShootCommand(manipulatorSubsystem));
            shooTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

            Trigger intakeTrigger = useXboxController ? xBoxController.y() : new Trigger(() -> joystick.getRawButton(10));
            intakeTrigger.onTrue(new IntakeCommand(manipulatorSubsystem));
            intakeTrigger.onFalse(new StopManipulatorCommand(manipulatorSubsystem));

            Trigger outtakeTrigger = useXboxController ? xBoxController.rightTrigger() : new Trigger(() -> joystick.getRawButton(9));
            outtakeTrigger.whileTrue(new OuttakeCommand(manipulatorSubsystem));

        }

        // if (enableClimb && false) {
        //     Trigger newClimbTrigger = controller.back();

        //      newClimbTrigger.onTrue(new ClimbCommand(climbSubsystem));
        //      newClimbTrigger.onFalse(new ClimbCommand(climbSubsystem));
            
        //     if (enableHinge) {
        //         newClimbTrigger.whileTrue(new ClimbWithArmCommand(hingeSubsystem));
        //     }
        // }

        if (enableClimb)
        {
            Trigger upClimbTrigger = useXboxController ? xBoxController.back() : new Trigger(() -> joystick.getRawButton(7));
            Trigger downClimbTrigger = useXboxController ? xBoxController.start() : new Trigger(() -> joystick.getRawButton(6));

            upClimbTrigger.onTrue(new ClimbCommandUp(climbSubsystem));
            upClimbTrigger.onTrue(new ClimbWithArmCommand(hingeSubsystem));

            downClimbTrigger.onTrue(new ClimbCommandDown(climbSubsystem));
            downClimbTrigger.onTrue(new SequentialCommandGroup(new WaitCommand(.75), new ClimbWithArmCommandDown(hingeSubsystem)));
        }

        if (enableGroundIntake) {
            // Trigger intakeTrigger = controller.rightTrigger();
            // intakeTrigger.whileTrue(new GroundIntakeCommand(groundIntakeSubsystem));
        }

        if (enableLEDs) {
            LEDs.setColor(0, 0, 255);
        }
        
        //   =========   OTHER BINDINGS   =========
    
    }

    public void configureForTeleop() {
        if (RobotContainer.enableHinge) {
            InitializeHinge(); // WHY is this done differently?!?!
        }
      
        configureButtonBindings();
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public void InitializeHinge() {
        new HingeInitializeCommand(hingeSubsystem).schedule();
    }

    public Command getAutonomousCommand(Autos auto) {
        // ally multiplier is used to set the proper direction for 
        //  red vs blue since they are mirrored
        int allyMultiplier = 1;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            allyMultiplier = ally.get() == Alliance.Blue ? 1 : -1;
        }

        Crashboard.toDashboard("Alliance: ", ally.get().toString(), "Competition");

        switch (auto) {

            case JUST_SHOOT:
            {
                //just shoot speaker
                return Commands.runOnce(() -> swerveSubsystem.zeroHeading())
                    .andThen( () -> swerveSubsystem.getKinematics().resetHeadings(new Rotation2d[] {
                        new Rotation2d(0), 
                        new Rotation2d(0),
                        new Rotation2d(0),
                        new Rotation2d(0)}))
                    .andThen(new AutoShootCommand(manipulatorSubsystem));
            }
                

            case FORWARD_SIDE:
            {
                return Commands.sequence(
                    new AutoShootCommand(manipulatorSubsystem),
                    new WaitCommand(1.0),
                    new DriveToPointCommand(swerveSubsystem, -1.5,0, 0.3)
                );
            }

            case SOURCE_SIDE:
            {

                //   Starting source side (automatically detects red or blue)
                return Commands.sequence(
                    new AutoShootCommand(manipulatorSubsystem),
                    new WaitCommand(1.0),
                    new DriveToPointCommand(swerveSubsystem, -1.7, 1.0 * allyMultiplier, 0.3),
                    new DriveToPointCommand(swerveSubsystem, -1.7, -1.7 * allyMultiplier, 0.3),
                    new DriveToPointCommand(swerveSubsystem, -30 * allyMultiplier, 0.15)
                );
            }


            case AMP_SIDE:
            {
                //shoot and taxi
                return new FollowTrajectoryCommand("TestPath", swerveSubsystem, false);
            }
        
            case DO_NOTHING:
            default:
            {
                return Commands.idle();
            }
        }

    }
}