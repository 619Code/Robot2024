package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants.ArmTargets;
import frc.robot.subsystems.ManipulatorSubsystem;

public class RevUpShooterCommand extends Command {

    private final ManipulatorSubsystem manipulatorSubsystem;
    private final ArmTargets targetPos;

    RevUpShooterCommand(ManipulatorSubsystem manipulatorSubsystem, ArmTargets targetPos) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.targetPos = targetPos;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void initialize() {
        if (ArmTargets.SPEAKER == targetPos) {
            manipulatorSubsystem.setShooterRPM(Constants.ManipulatorConstants.passerShooterVelocityToReachBeforeFeedingNote);
        } else {
            manipulatorSubsystem.setShooterRPM(Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote);
        }
    }

    @Override
    public void execute() {
        // Nothing to do here.
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to do here.
    }

    @Override
    public boolean isFinished() {
        if (ArmTargets.SPEAKER == targetPos) {
            return manipulatorSubsystem.getShooterRPM().gt(Constants.ManipulatorConstants.passerShooterVelocityToReachBeforeFeedingNote);
        } else {
            return manipulatorSubsystem.getShooterRPM().gt(Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote);
        }
    }
}

