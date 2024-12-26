package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCommandSequence extends SequentialCommandGroup {

    ShootCommandSequence(ManipulatorSubsystem manipulatorSubsystem) {
        addRequirements(manipulatorSubsystem);

        // Create the sequences
        super.addCommands(
            new RevUpShooterCommand(
                manipulatorSubsystem,
                Constants.ManipulatorConstants.ArmTargets.SPEAKER
            ),
            new ShootCommand(manipulatorSubsystem),
            new SimulateNoteCommand(null, null)
        );
    }
}
