package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public ShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spinShooter(Constants.ManipulatorConstants.outtakeSpeed); // test value, make sure to change once we g
        subsystem.spintake(Constants.ManipulatorConstants.intakeSpeedWhenOuttaking);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}