package org.usfirst.frc.team199.Robot2018.pathfinder;

import org.usfirst.frc.team199.Robot2018.pathfinder.followers.EncoderFollower;

public class PathfinderJNI {

    static boolean libLoaded = false;

    static {
        if (!libLoaded) {
            try {
                System.loadLibrary("pathfinderjava");
            } catch (Exception e) {
                e.printStackTrace();
            }
            libLoaded = true;
        }
    }

    public static Trajectory generateTrajectory(Waypoint[] waypoints, Trajectory.Config c) {
        return new Trajectory(generateTrajectory(waypoints, c.fit, c.sample_count, c.dt, c.max_velocity, c.max_acceleration, c.max_jerk));
    }
    public static native Trajectory.Segment[] generateTrajectory(Waypoint[] waypoints, Trajectory.FitMethod fit, int samples, double dt, double max_velocity, double max_acceleration, double max_jerk);

    public static Trajectory[] modifyTrajectoryTank(Trajectory traj, double wheelbase_width) {
        Trajectory.Segment[][] mod = modifyTrajectoryTank(traj.segments, wheelbase_width);
        return new Trajectory[] { new Trajectory(mod[0]), new Trajectory(mod[1]) };
    }
    public static native Trajectory.Segment[][] modifyTrajectoryTank(Trajectory.Segment[] source, double wheelbase_width);

    public static native void trajectorySerialize(Trajectory.Segment[] source, String filename);
    public static native Trajectory.Segment[] trajectoryDeserialize(String filename);

    public static native void trajectorySerializeCSV(Trajectory.Segment[] source, String filename);
    public static native Trajectory.Segment[] trajectoryDeserializeCSV(String filename);
}
