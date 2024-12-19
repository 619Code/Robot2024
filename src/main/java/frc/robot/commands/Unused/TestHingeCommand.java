package frc.robot.commands.Unused;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HingeSubsystem;

public class TestHingeCommand extends Command {
    HingeSubsystem subsystem;
    double speed = 0;
    CommandXboxController controller;
    DoubleSupplier speedSupplier;

    public TestHingeCommand(HingeSubsystem subsystem, DoubleSupplier speedSupplier) {
        this.subsystem = subsystem;
        this.speedSupplier = speedSupplier;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {       
    }

    @Override
    public void execute() {
        speed = speedSupplier.getAsDouble();
        if (Math.abs(speed) < 0.1) speed = 0;
        subsystem.spinge(speed * 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopHinge();
    }
}
