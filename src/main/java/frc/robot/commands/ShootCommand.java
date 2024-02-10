package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public ShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spinShooter(-.05);
    }
}
