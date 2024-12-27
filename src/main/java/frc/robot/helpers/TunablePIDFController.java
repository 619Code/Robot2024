package frc.robot.helpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunablePIDFController {
    private final String controllerName;
    private final PIDController pidController;
    private SimpleMotorFeedforward ffController;
    private double setpoint = 0.0;
    private boolean isTuning = false;

    // TODO: Make this a struct
    private final DoubleEntry KpEntry;
    private final DoubleEntry KiEntry;
    private final DoubleEntry KdEntry;
    private final DoubleEntry KsEntry;
    private final DoubleEntry KvEntry;
    private final DoubleEntry KaEntry;
    private final DoublePublisher setpointPublisher;
    private final DoublePublisher errorPublisher;
    private final DoublePublisher outputPIDPublisher;
    private final DoublePublisher outputFFPublisher;
    private final DoublePublisher outputPublisher;


    public TunablePIDFController(
        String controllerName,
        double Kp,
        double Ki,
        double Kd,
        double Ks,
        double Kv,
        double Ka
    ) {
        this.controllerName = controllerName;
        pidController = new PIDController(Kp, Ki, Kd);
        ffController = new SimpleMotorFeedforward(Ks, Kv, Ka);
        setpoint = 0.0;

        KpEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Kp").getEntry(Kp);
        KiEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Ki").getEntry(Ki);
        KdEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Kd").getEntry(Kd);
        KsEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Ks").getEntry(Ks);
        KvEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Kv").getEntry(Kv);
        KaEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Ka").getEntry(Ka);
        setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/setpoint").publish();
        errorPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/error").publish();
        outputPIDPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/PID Output").publish();
        outputFFPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/FF Output").publish();
        outputPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Output").publish();

        KpEntry.set(Kp);
        KiEntry.set(Ki);
        KdEntry.set(Kd);
        KsEntry.set(Ks);
        KvEntry.set(Kv);
        KaEntry.set(Ka);
        setpointPublisher.set(0.0);
        errorPublisher.set(0.0);
        outputPIDPublisher.set(0.0);
        outputFFPublisher.set(0.0);
        outputPublisher.set(0.0);

    }

    public TunablePIDFController(
        String controllerName,
        double Kp,
        double Ki,
        double Kd
    ) {
        this.controllerName = controllerName;
        pidController = new PIDController(Kp, Ki, Kd);
        ffController = new SimpleMotorFeedforward(0, 0, 0);
        setpoint = 0.0;

        KpEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Kp").getEntry(Kp);
        KiEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Ki").getEntry(Ki);
        KdEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Kd").getEntry(Kd);
        KsEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Ks").getEntry(0);
        KvEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Kv").getEntry(0);
        KaEntry = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Ka").getEntry(0);
        setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/setpoint").publish();
        errorPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/error").publish();
        outputPIDPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/PID Output").publish();
        outputFFPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/FF Output").publish();
        outputPublisher = NetworkTableInstance.getDefault().getDoubleTopic(controllerName + "/Output").publish();

        KpEntry.set(Kp);
        KiEntry.set(Ki);
        KdEntry.set(Kd);
        KsEntry.set(0.0);
        KvEntry.set(0.0);
        KaEntry.set(0.0);
        setpointPublisher.set(0.0);
        errorPublisher.set(0.0);
        outputPIDPublisher.set(0.0);
        outputFFPublisher.set(0.0);
        outputPublisher.set(0.0);

    }

    public void setTuningMode(boolean isTuning) {
        this.isTuning = isTuning;
    }

    public void setKp(double Kp) {
        pidController.setP(Kp);
        KpEntry.set(Kp);
    }

    public void setKi(double Ki) {
        pidController.setI(Ki);
        KiEntry.set(Ki);
    }

    public void setKd(double Kd) {
        pidController.setD(Kd);
        KdEntry.set(Kd);
    }

    public void setPID(double Kp, double Ki, double Kd) {
        KpEntry.set(Kp);
        KiEntry.set(Ki);
        KpEntry.set(Kd);

        pidController.setPID(Kp, Ki, Kd);
    }

    /**
     * This updates the controller with the current values in
     * network tables
     */
    public void updatePID() {
        pidController.setPID(
            KpEntry.get(),
            KiEntry.get(),
            KdEntry.get()
        );
    }

    /**
     * WARNING: This makes an allocation. DO NOT CALL IN A LOOP!
     * @param Ks
     * @param Kv
     * @param Ka
     */
    public void setFF(double Ks, double Kv, double Ka) {
        // The FF values aren't supposed to be changed,
        // but in case we want to, this mechanism allows for it
        KsEntry.set(Ks);
        KvEntry.set(Kv);
        KaEntry.set(Ka);

        ffController = new SimpleMotorFeedforward(Ks, Kv, Ka);
    }

    /**
     * Updates the Ks, Kv, and Ka values from network tables
     *
     * WARNING: This makes an allocation. DO NOT CALL IN A LOOP!
     */
    public void updateFF() {
        ffController = new SimpleMotorFeedforward(
            KsEntry.get(),
            KvEntry.get(),
            KaEntry.get()
        );
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setSetpoint(setpoint);
        setpointPublisher.set(pidController.getSetpoint());
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double calculate(double measurement) {
        if (measurement < 1e-6) {
            measurement = 0;
        }

        if (isTuning) {
            updatePID();
        }

        double pidOut = pidController.calculate(measurement);
        double ffOut = ffController.calculate(setpoint);

        errorPublisher.set(pidController.getPositionError());
        outputPIDPublisher.set(pidOut);
        outputFFPublisher.set(ffOut);
        outputPublisher.set(pidOut + ffOut);

        return pidOut + ffOut;
    }

    public double calculate(double measurement, double setpoint) {
        if (measurement < 1e-6) {
            measurement = 0;
        }

        if (isTuning) {
            updatePID();
        }

        setSetpoint(setpoint);

        double pidOut = pidController.calculate(measurement);
        double ffOut = ffController.calculate(setpoint);

        errorPublisher.set(pidController.getPositionError());
        outputPIDPublisher.set(pidOut);
        outputFFPublisher.set(ffOut);
        outputPublisher.set(pidOut + ffOut);

        return pidOut + ffOut;
    }

}
