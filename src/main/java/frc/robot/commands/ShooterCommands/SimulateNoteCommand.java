package frc.robot.commands.ShooterCommands;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// Based off the example code: https://github.com/Mechanical-Advantage/AdvantageKit/blob/18a0219f60108e3dc1e8512d59fcba0e657770af/example_projects/kitbot_2024/src/main/java/frc/robot/util/NoteVisualizer.javahttps://github.com/Mechanical-Advantage/AdvantageKit/blob/18a0219f60108e3dc1e8512d59fcba0e657770af/example_projects/kitbot_2024/src/main/java/frc/robot/util/NoteVisualizer.java

public class SimulateNoteCommand extends Command {

    private final StructArrayPublisher<Pose3d> posePublisher;
    private final Timer timer;
    private final double dt = 0.02;

    private Pose3d[] trajectory;
    private final Pose3d initialPose;
    private final Measure<Velocity<Distance>> initialVelocity;

    public SimulateNoteCommand(Pose3d initialPose,  Measure<Velocity<Distance>> launchVelocity) {
        posePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Note", Pose3d.struct).publish();
        posePublisher.set(new Pose3d[0]);

        this.initialPose = initialPose;
        this.initialVelocity = launchVelocity;
        this.timer = new Timer();
    }

    // public SimulateNoteCommand(SwerveSubsystem swerveSubsystem, ManipulatorSubsystem manipulatorSubsystem) {

    // }

    private Pose3d[] caculateTrajectory(Pose3d initalPose, Measure<Velocity<Distance>> initialSpeed) {
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
        double rz = initalPose.getRotation().getZ();
        double ry = initalPose.getRotation().getY();
        double v_x = initialSpeed.magnitude() * Math.cos(rz);
        double v_y = initialSpeed.magnitude() * Math.sin(rz);
        double v_z = initialSpeed.magnitude() * Math.cos(ry);
        double x_0_x = initalPose.getX() * Math.cos(rz);
        double x_0_z = initalPose.getZ();


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

            trajectory.add(new Pose3d(
                new Translation3d(
                    Units.Meters.of(x_t * Math.cos(rz)),
                    Units.Meters.of(x_t * Math.sin(rz)),
                    Units.Meters.of(z_t)
                ),
                new Rotation3d(
                    0,
                    -Math.atan2(verticalMag - 9.8*t, horizontalMag), // The axis of rotation for the note is different from how I'd have thought
                    initalPose.getZ()
                )
            ));
        }

        return (Pose3d[])trajectory.toArray(new Pose3d[trajectory.size()]);
    }

    @Override
    public void initialize() {
        trajectory = caculateTrajectory(initialPose, initialVelocity);

        posePublisher.set(Arrays.copyOfRange(trajectory, 0, 0));

        timer.start();
    }

    @Override
    public void execute() {
        // There may be an off by 1 here, but it looks close enough
        int index = (int)(timer.get() / dt);
        if (index < trajectory.length) {
            posePublisher.set(Arrays.copyOfRange(trajectory, 0, index));
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        int index = (int)(timer.get() / dt);
        return index > trajectory.length;
    }

}
