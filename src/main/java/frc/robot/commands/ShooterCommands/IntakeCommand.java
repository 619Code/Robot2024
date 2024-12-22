package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCommand extends Command{

    private ManipulatorSubsystem subsystem;

    public IntakeCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // test value plz change.
        // NO! I WON'T!
        subsystem.setShooterRPM(Constants.ManipulatorConstants.shooterIntakingRPM);
    }

    @Override
    public boolean isFinished() {
        return subsystem.isIntakeTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopIntake();
    }
}
