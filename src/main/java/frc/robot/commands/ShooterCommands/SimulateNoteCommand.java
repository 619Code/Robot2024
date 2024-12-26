package frc.robot.commands.ShooterCommands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// Based off the example code: https://github.com/Mechanical-Advantage/AdvantageKit/blob/18a0219f60108e3dc1e8512d59fcba0e657770af/example_projects/kitbot_2024/src/main/java/frc/robot/util/NoteVisualizer.javahttps://github.com/Mechanical-Advantage/AdvantageKit/blob/18a0219f60108e3dc1e8512d59fcba0e657770af/example_projects/kitbot_2024/src/main/java/frc/robot/util/NoteVisualizer.java

public class SimulateNoteCommand extends Command {

    private final StructArrayPublisher<Pose3d> trajectoryPublisher;
    private final StructPublisher<Pose3d> notePublisher;
    private final SwerveSubsystem swerveSubsystem;
    private final HingeSubsystem hingeSubsystem;
    private final Timer timer;
    private final double dt = 0.02;

    private Pose3d[] trajectory;

    public SimulateNoteCommand(SwerveSubsystem swerveSubsystem, HingeSubsystem hingeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.hingeSubsystem = hingeSubsystem;
        this.timer = new Timer();

        trajectoryPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("NoteTrajectory", Pose3d.struct).publish();
        trajectoryPublisher.set(new Pose3d[0]);

        notePublisher = NetworkTableInstance.getDefault().getStructTopic("Note", Pose3d.struct).publish();
        notePublisher.set(new Pose3d());


        // We intentionally don't call addRequirement here,
        // because we want this to run while those subsystems are executing other commands
    }

    private boolean withinField(Pose3d pose) {
        return  pose.getX() > 0 &&
                pose.getY() > 0 &&
                pose.getX() < Units.Meters.convertFrom(52, Units.Feet) &&
                pose.getY() < Units.Meters.convertFrom(26, Units.Feet);
    }

    private Pose3d[] caculateTrajectory(Pose3d initialPose, Measure<Velocity<Distance>> initialSpeed) {
        /*
         * Naive strategy:
         * 1. Move into the trajectory plane
         * 2. Use classial mechanics to find all the points.
         *    The rotations will follow the velocity vector
         * 3. Convert the points in that plane back into the 3D field frame.
         *
         * x_x = v_0*t
         * x_y = v_0*t -g/2 * t^2
         * v_x = v_0
         * v_y = v_0 - g*t
         */
        double rz = initialPose.getRotation().getZ();
        double ry = initialPose.getRotation().getY();
        double rx = initialPose.getRotation().getX();

        double v_x = initialSpeed.magnitude() * Math.cos(rz) * Math.cos(ry);
        double v_y = initialSpeed.magnitude() * Math.sin(rz) * Math.cos(ry);
        double v_z = initialSpeed.magnitude() * Math.sin(ry);
        double x_0_x = 0.0;
        double x_0_z = initialPose.getZ();


        // Project the launch vector into the launch plane
        // the horizontal vector (b) is the unit vector in the xy plane
        double b_x = Math.cos(rz);
        double b_y = Math.sin(rz);

        double horizontalMag = v_x*b_x + v_y*b_y;
        double verticalMag = v_z;

        double endTime = (v_z + Math.sqrt(v_z*v_z + 2*x_0_z*9.8))/9.8;

        ArrayList<Pose3d> trajectory = new ArrayList<Pose3d>((int)(endTime/0.02));
        for (double t = 0.0; t < endTime; t += dt) {
            double x_t = x_0_x + horizontalMag*t;
            double z_t = x_0_z + verticalMag*t - 4.9*t*t;
            double v_x_t = horizontalMag;
            double v_z_t = verticalMag - 9.8*t;

            // Put things back in Field coords
            double roll = 0;
            double pitch = -Math.atan2(v_z_t, v_x_t);
            Pose3d pose = new Pose3d(
                new Translation3d(
                    Units.Meters.of(initialPose.getX()).plus(Units.Meters.of(x_t * Math.cos(rz))),
                    Units.Meters.of(initialPose.getY()).plus(Units.Meters.of(x_t * Math.sin(rz))),
                    Units.Meters.of(z_t)
                ),
                new Rotation3d(
                    roll,
                    pitch,
                    //-Math.atan2(verticalMag - 9.8*t, horizontalMag), // The axis of rotation for the note is different from how I'd have thought
                    initialPose.getRotation().getZ()
                )
            );

            trajectory.add(pose);

            // Our simple physics sim is to consider the note scored when it leaves the field
            if (!withinField(pose)) {
                break;
            }
        }

        return (Pose3d[])trajectory.toArray(new Pose3d[trajectory.size()]);
    }

    @Override
    public void initialize() {
        // TODO: These use SPEAKER defaults for now, because
        // the hinge isn't simulated/animated yet
        Pose3d initialPose = new Pose3d(
            new Translation3d(
                swerveSubsystem.getPose2d().getX(),
                swerveSubsystem.getPose2d().getY(),
                0.6 // This should be 1 of two values depending on angle of the hinge
            ),
            // We do a bad thing here. This rotation is in Robot coords, not Field coords.
            new Rotation3d(
                0,
                Units.Radians.convertFrom(55, Units.Degrees), //Units.Radians.convertFrom(hingeSubsystem.getAbsoluteDegrees(), Units.Degrees),
                Units.Radians.convertFrom(swerveSubsystem.getHeadingDegrees(), Units.Degrees)
            )
        );
        // TODO: This is a guess. It will vary depending on AMP or SPEAKER
        Measure<Velocity<Distance>> initialVelocity = Units.MetersPerSecond.of(10.0);

        trajectory = caculateTrajectory(initialPose, initialVelocity);

        trajectoryPublisher.set(Arrays.copyOfRange(trajectory, 0, 0));
        notePublisher.set(trajectory[0]);

        timer.start();
    }

    @Override
    public void execute() {
        // There may be an off by 1 here, but it looks close enough
        int index = (int)(timer.get() / dt);
        if (index < trajectory.length) {
            trajectoryPublisher.set(Arrays.copyOfRange(trajectory, 0, index));
            notePublisher.set(trajectory[index]);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        int index = (int)(timer.get() / dt);
        return index > trajectory.length;
    }

}
