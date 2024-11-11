package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.Robot;
import frc.robot.helpers.Crashboard;

public class ManipulatorSubsystem extends SubsystemBase {

    public final CANSparkMax intakeLeader;
    public final CANSparkMax shooterLeader;

    public final DigitalInput intakeProximitySensor;

    public final RelativeEncoder shooterEncoder;

    private final DCMotorSim shooterSim;
    private final DCMotorSim intakeSim;

    private PIDController shooterOnboardPID;
    private SimpleMotorFeedforward shooterFeedforward;

    public ManipulatorSubsystem() {
        intakeLeader = new CANSparkMax(Constants.ManipulatorConstants.kIntakeLeaderPort, MotorType.kBrushless);
        intakeLeader.restoreFactoryDefaults();
        intakeLeader.setIdleMode(IdleMode.kBrake);
        intakeLeader.setSmartCurrentLimit(35);
        intakeLeader.setInverted(Constants.ManipulatorConstants.kInakeLeaderInverted);

        shooterLeader = new CANSparkMax(Constants.ManipulatorConstants.kShooterLeaderPort, MotorType.kBrushless);
        shooterLeader.restoreFactoryDefaults();
        shooterLeader.setIdleMode(IdleMode.kBrake);        
        shooterLeader.setSmartCurrentLimit(35);
        shooterLeader.setInverted(Constants.ManipulatorConstants.kShooterLeaderInverted);

        intakeProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kIntakeSensorPort);

        shooterEncoder = this.shooterLeader.getEncoder();

        shooterSim = new DCMotorSim(
            DCMotor.getNEO(1), 
            1.0,
            0.000418
        );

        intakeSim = new DCMotorSim(
            DCMotor.getNEO(1), 
            1.0,
            0.000418
        );

        this.initPIDs();

    }

    public void setShooterSpeedByRPM(double speed) {
        if (Robot.isReal()) {
            speed = speed/60.0;
            shooterLeader.setVoltage(shooterOnboardPID.calculate(speed) + shooterFeedforward.calculate(speed));
        } else {
            shooterSim.setInputVoltage(shooterOnboardPID.calculate(speed) + shooterFeedforward.calculate(speed));
        }
    }

    public double getShooterRPM() {
        if (Robot.isReal()) {
            return shooterEncoder.getVelocity();
        } else {
            return shooterSim.getAngularVelocityRPM();
        }
        
    }

    public void initPIDs() {
        shooterOnboardPID = new PIDController(Constants.ManipulatorConstants.SHOOTER_KP, Constants.ManipulatorConstants.SHOOTER_KI, Constants.ManipulatorConstants.SHOOTER_KD);
        shooterFeedforward = new SimpleMotorFeedforward(Constants.ManipulatorConstants.SHOOTER_KS, Constants.ManipulatorConstants.SHOOTER_KV, Constants.ManipulatorConstants.SHOOTER_KA);
    }

    @Override 
    public void simulationPeriodic() {
        shooterSim.update(0.02);
        intakeSim.update(0.02);
    }

    @Override
    public void periodic() {
        
        Crashboard.toDashboard("Sensor value: ", intakeProximitySensor.get(), "Manipulator");
        OurRobotState.hasNote = !intakeProximitySensor.get();
    }

    public double GetShooterVelocity(){
        if (Robot.isReal()) {
            return shooterEncoder.getVelocity();
        } else {
            return shooterSim.getAngularVelocityRPM();
        }
    }

    public void spintake(double speed) {
        if (Robot.isReal()) {
            intakeLeader.set(speed);
        } else {
            intakeSim.setInputVoltage(speed * RobotController.getBatteryVoltage());
        }
    }

    public void spintakeVoltage(double voltage) {
        if (Robot.isReal()) {
            intakeLeader.setVoltage(voltage);
        } else {
            intakeSim.setInputVoltage(voltage);
        }
    }

    public void spinShooterVoltage(double voltage) {
        if (Robot.isReal()) {
            shooterLeader.setVoltage(voltage);
        } else {
            shooterSim.setInputVoltage(voltage);
        }
    }

    public void spinShooter(double speed) {
        if (Robot.isReal()) {
            shooterLeader.set(speed);
        } else {
            shooterSim.setInputVoltage(speed * RobotController.getBatteryVoltage());
        }
    }

    public boolean intakeTrigged() {
        return !intakeProximitySensor.get();
    }

    public void stopIntake(){
        if (Robot.isReal()) {
            intakeLeader.stopMotor();
        } else {
            intakeSim.setInputVoltage(0);
        }
    }

    public void stopShooter(){
        if (Robot.isReal()) {
            shooterLeader.stopMotor();
        } else {
            shooterSim.setInputVoltage(0);
        }
    }

    public void stopAll(){
        intakeLeader.stopMotor();
        shooterLeader.stopMotor();
    }  
}
