package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootCommandSequence extends SequentialCommandGroup {

    public ShootCommandSequence(
        SwerveSubsystem swerveSubsystem,
        ManipulatorSubsystem manipulatorSubsystem,
        HingeSubsystem hingeSubsystem
    ) {
        // Create the sequences
        super(
            new RevUpShooterCommand(
                manipulatorSubsystem,
                Constants.ManipulatorConstants.ArmTargets.SPEAKER
            ),
            new ShootCommand(manipulatorSubsystem),
            new SimulateNoteCommand(swerveSubsystem, hingeSubsystem),
\        );
    }
}
