package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCommand extends Command{

    private ManipulatorSubsystem subsystem;
    private boolean hasReachedVelocity = false;

    private double outtakeSpeed;
    private double intakeSpeed;
    private int  RPMsRequiredForOuttake;
    
    public ShootCommand(ManipulatorSubsystem subsystem, double _outtakeSpeed, double _intakeSpeed, int _RPMsRequiredForOuttake) {
        this.subsystem = subsystem;

        this.outtakeSpeed = _outtakeSpeed;
        this.intakeSpeed = _intakeSpeed;
        this.RPMsRequiredForOuttake = _RPMsRequiredForOuttake;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spinShooter(this.outtakeSpeed); // test value, make sure to change once we g
      //  subsystem.spintake(Constants.ManipulatorConstants.intakeSpeedWhenOuttaking);
    }

    @Override
    public void execute() {
        if(subsystem.GetShooterVelocity() >= this.RPMsRequiredForOuttake){

            hasReachedVelocity = true;

        }

        if(hasReachedVelocity){

            subsystem.spintake(this.intakeSpeed);

        }
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        hasReachedVelocity = false;
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}
