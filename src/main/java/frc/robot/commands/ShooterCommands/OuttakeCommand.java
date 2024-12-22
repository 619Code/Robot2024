package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class OuttakeCommand extends Command {

    private ManipulatorSubsystem subsystem;

    public OuttakeCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setShooterRPM(Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote.times(-0.5));
        subsystem.setIntakePercentOut(Constants.ManipulatorConstants.intakePercentOutWhenOuttaking * -0.5);
    }

    @Override
    public void execute() {
        // We don't update anything, so ignore?
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopAll();
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isIntakeTriggered();
    }



}
