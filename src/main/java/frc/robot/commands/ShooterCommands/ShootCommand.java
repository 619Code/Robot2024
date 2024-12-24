package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.helpers.Crashboard;
import frc.robot.helpers.NamedUnits.PercentOutput;
import frc.robot.helpers.NamedUnits.RevolutionsPerMinute;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCommand extends Command {

    private ManipulatorSubsystem subsystem;
    private boolean hasReachedVelocity = false;

    private PercentOutput intakePercentOut;
    private RevolutionsPerMinute RPMRequiredForOuttake;

    public ShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (OurRobotState.currentArmPosition == ArmPosEnum.AMP) {

            this.intakePercentOut = Constants.ManipulatorConstants.intakePercentOutWhenOuttaking;
            this.RPMRequiredForOuttake = Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.SPEAKER) {

            this.intakePercentOut = Constants.ManipulatorConstants.intakePercentOutWhenOuttaking;
            this.RPMRequiredForOuttake = Constants.ManipulatorConstants.speakerShooterVelocityToReachBeforeFeedingNote;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.LONG_SHOT) {
            this.intakePercentOut = Constants.ManipulatorConstants.intakePercentOutWhenOuttaking;
            this.RPMRequiredForOuttake = Constants.ManipulatorConstants.passerShooterVelocityToReachBeforeFeedingNote;
        } else {
            // do nothing, no shooting!
            // Shooter, no shooting!
            this.intakePercentOut = new PercentOutput(0);
            this.RPMRequiredForOuttake = new RevolutionsPerMinute(0);
        }

        subsystem.setShooterRPM(RPMRequiredForOuttake);
    }

    @Override
    public void execute() {

        Crashboard.toDashboard("shooter flywheel RPMS: ", subsystem.getShooterRPM().magnitude(), "shooter");

        if(subsystem.getShooterRPM().gte(RPMRequiredForOuttake.times(0.9))){

            hasReachedVelocity = true;

        }

        if(hasReachedVelocity){

            subsystem.setIntakePercentOut(this.intakePercentOut);

        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        hasReachedVelocity = false;
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}