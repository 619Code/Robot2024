package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.Robot;
import frc.robot.helpers.Crashboard;
import frc.robot.helpers.NamedUnits.PercentOutput;
import frc.robot.helpers.NamedUnits.RevolutionsPerMinute;

public class ManipulatorSubsystem extends SubsystemBase {
    private final boolean enabled;

    private final CANSparkMax intakeLeader;
    private final CANSparkMax shooterLeader;

    private final DigitalInput intakeProximitySensor;

    private final RelativeEncoder shooterEncoder;

    private final DCMotorSim shooterSim;
    private final DCMotorSim intakeSim;

    private final PIDController shooterOnboardPID;
    private final SimpleMotorFeedforward shooterFeedforward;
    private final MutableMeasure<Velocity<Angle>> shooterRPMSetpoint = MutableMeasure.zero(Units.RPM);

    public ManipulatorSubsystem(boolean enabled) {
        this.enabled = enabled;

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

        shooterOnboardPID = new PIDController(
            Constants.ManipulatorConstants.SHOOTER_KP,
            Constants.ManipulatorConstants.SHOOTER_KI,
            Constants.ManipulatorConstants.SHOOTER_KD
        );
        shooterFeedforward = new SimpleMotorFeedforward(
            Constants.ManipulatorConstants.SHOOTER_KS,
            Constants.ManipulatorConstants.SHOOTER_KV,
            Constants.ManipulatorConstants.SHOOTER_KA
        );

        shooterOnboardPID.setSetpoint(shooterRPMSetpoint.magnitude());
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

        // Update our PID controller
        if (enabled) {
            if (Robot.isReal()) {
                shooterLeader.setVoltage(shooterOnboardPID.calculate(shooterEncoder.getVelocity()) + shooterFeedforward.calculate(shooterRPMSetpoint.magnitude()));
            } else {
                shooterSim.setInputVoltage(shooterOnboardPID.calculate(shooterSim.getAngularVelocityRPM()) + shooterFeedforward.calculate(shooterRPMSetpoint.magnitude()));
            }
        }
    }

    public RevolutionsPerMinute getShooterRPM() {
        if (Robot.isReal()) {
            return new RevolutionsPerMinute(shooterEncoder.getVelocity());
        } else {
            return new RevolutionsPerMinute(shooterSim.getAngularVelocityRPM());
        }
    }

    public void setShooterRPM(RevolutionsPerMinute rpm) {
        if (enabled) {
            // Update the setpoint. The actual motor is controlled
            // in the periodic loop so we get continous feedback
            shooterRPMSetpoint.mut_replace(rpm);
            shooterOnboardPID.setSetpoint(shooterRPMSetpoint.magnitude());
        }
    }

    public void setShooterVoltage(Measure<Voltage> voltage) {
        if (enabled) {
            if (Robot.isReal()) {
                shooterLeader.setVoltage(voltage.magnitude());
            } else {
                shooterSim.setInputVoltage(voltage.magnitude());
            }
        }
    }

    /**
     * @param value Should be [-1.0, 1.0]
     */
    public void setIntakePercentOut(PercentOutput value) {
        if (enabled) {
            if (Robot.isReal()) {
                intakeLeader.set(value.magnitude());
            } else {
                intakeSim.setInputVoltage(value.magnitude() * RobotController.getBatteryVoltage());
            }
        }
    }

    public void setIntakeVoltage(Measure<Voltage> voltage) {
        if (enabled) {
            if (Robot.isReal()) {
                intakeLeader.setVoltage(voltage.magnitude());
            } else {
                intakeSim.setInputVoltage(voltage.magnitude());
            }
        }
    }

    public boolean isIntakeTriggered() {
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
