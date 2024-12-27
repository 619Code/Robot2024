package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.helpers.Crashboard;
import frc.robot.helpers.NamedUnits.PercentOutput;
import frc.robot.helpers.NamedUnits.RevolutionsPerMinute;
import frc.robot.subsystems.ManipulatorSubsystem;


public class ShootCommand extends Command {

    private final ManipulatorSubsystem subsystem;
    private final Timer timer;

    private PercentOutput intakePercentOut;

    public ShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;
        timer = new Timer();

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Determine the speed to feed the note into the shooter.

        if (OurRobotState.currentArmPosition == ArmPosEnum.AMP) {

            this.intakePercentOut = Constants.ManipulatorConstants.intakePercentOutWhenOuttaking;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.SPEAKER) {

            this.intakePercentOut = Constants.ManipulatorConstants.intakePercentOutWhenOuttaking;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.LONG_SHOT) {
            this.intakePercentOut = Constants.ManipulatorConstants.intakePercentOutWhenOuttaking;
        } else {
            // do nothing, no shooting!
            // Shooter, no shooting!
            this.intakePercentOut = new PercentOutput(0);
        }

        // Kick the note out.
        subsystem.setIntakePercentOut(intakePercentOut);
        timer.start();
    }

    @Override
    public void execute() {
        // Condition checking done in isFinished
    }

    @Override
    public boolean isFinished() {
        // Give a little time for the motors to spin the note out.
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}