package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import frc.robot.Constants;
=======
>>>>>>> main
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public IntakeCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
<<<<<<< HEAD
        subsystem.spintake(Constants.ManipulatorConstants.intakeSpeed); // test value plz change. NO! I WON'T!
    }

    @Override
    public boolean isFinished() {
        System.out.println(subsystem.intakeTrigged());
        return subsystem.intakeTrigged(); 
=======
        subsystem.spintake(-.05); // test value plz change
>>>>>>> main
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopIntake();
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> main
