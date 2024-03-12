package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.ManipulatorSubsystem;
 
public class AutoShootCommand extends Command {

    private ManipulatorSubsystem subsystem;
    private boolean hasReachedVelocity = false;

    private double outtakeSpeed;
    private double intakeSpeed;
    private int  RPMsRequiredForOuttake;
    private Timer timer;
    
    public AutoShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;
        timer = new Timer();
        timer.reset();

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (OurRobotState.currentArmPosition == ArmPosEnum.AMP) {

            this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedAmp;
            this.intakeSpeed = Constants.ManipulatorConstants.intakeSpeedWhenOuttaking;
            this.RPMsRequiredForOuttake = Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.SPEAKER) {

            this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedSpeaker;
            this.intakeSpeed = Constants.ManipulatorConstants.intakeSpeedWhenOuttaking;
            this.RPMsRequiredForOuttake = Constants.ManipulatorConstants.speakerShooterVelocityToReachBeforeFeedingNote;

        } else {
            // do nothing, no shooting!
                // Shooter, no shooting!
            this.outtakeSpeed = 0;
            this.intakeSpeed = 0;
            this.RPMsRequiredForOuttake = 0;
        }

        subsystem.spinShooter(this.outtakeSpeed); // test value, make sure to change once we g
    }

    @Override
    public void execute() {
        if(subsystem.GetShooterVelocity() >= this.RPMsRequiredForOuttake){

            hasReachedVelocity = true;
            timer.start();

        }

        if(hasReachedVelocity){

            subsystem.spintake(this.intakeSpeed);

        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(5)) {
            timer.stop();
            timer.reset();
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        hasReachedVelocity = false;
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}