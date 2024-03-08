package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class StopManipulatorCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public StopManipulatorCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}