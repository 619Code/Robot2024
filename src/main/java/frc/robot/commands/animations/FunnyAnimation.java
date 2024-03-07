package frc.robot.commands.animations;

import frc.robot.Constants;
import frc.robot.subsystems.ledSubsystem;


public class FunnyAnimation implements CARAnimation{

    ledSubsystem subsystem;

    public FunnyAnimation(ledSubsystem _subsystem){

        this.subsystem = _subsystem;

    }

    
    public void start(){

    }

    public void update(){
        for(int i = 0; i < Constants.LEDConstants.ledCount; i++){

            this.subsystem.setColor((int)(Math.random() * 255), (int)(Math.random() * 255), (int)(Math.random() * 255), 0, i, 1);

        }
    }
}