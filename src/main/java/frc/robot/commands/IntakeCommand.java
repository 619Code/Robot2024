package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public IntakeCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spintake(-.05);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopIntake();
    }
}
