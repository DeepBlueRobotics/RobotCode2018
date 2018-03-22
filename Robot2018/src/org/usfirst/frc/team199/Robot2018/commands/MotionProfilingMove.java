package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.LiftHeight;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 *
 */
public class MotionProfilingMove extends Command {

	private double target;
	private DrivetrainInterface dt;
	private PIDSource avg;
	private SmartDashboardInterface sd;
	private double[] point;
	private boolean usePoint;
	Trajectory traj;
	int i;
	
	public MotionProfilingMove(double target, double[] point, DrivetrainInterface dt, SmartDashboardInterface sd,
			PIDSource avg, boolean usePoint) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.point = point;
		this.target = target;
		this.usePoint = usePoint;
		this.dt = dt;
		this.avg = avg;
		this.sd = sd;
		if (Robot.dt != null) {
			requires(Robot.dt);
		}
	}

	public MotionProfilingMove(double target, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource avg) {
		this(target, null, dt, sd, avg, false);
	}

	public MotionProfilingMove(double[] point, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource avg) {
		this(0, point, dt, sd, avg, true);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (!usePoint) {
			// use point because easier for motion profiling
			point[0] = 0;
			point[1] = target;
		}
		Waypoint[] pt = new Waypoint[1];
		pt[0] = new Waypoint(point[0], point[1], 0);
		Trajectory.FitMethod fit = Robot.getBool("MP_CubicTraj", true) ? Trajectory.FitMethod.HERMITE_CUBIC : Trajectory.FitMethod.HERMITE_QUINTIC;
		//0 samples is fast (1k), 1 is low (10k), and 2 is fast (100k)
		int samples = findSamples(Robot.getConst("MP_AmtSamples", 0));
//		int samples = (int) Robot.getConst("MP_AmtSamples", 1000);
		double timestep = Robot.rmap.getCycleTime();
		double maxVelocity = Robot.dt.getCurrentMaxSpeed();
		LiftHeight currPos = Robot.lift.getCurrPos();
		if (Robot.auto && currPos != LiftHeight.GROUND && currPos != LiftHeight.HOLD_CUBE) maxVelocity *= Robot.getConst("Auto Max Speed", 0.5);
		double maxAccel = Robot.rmap.gravity;
			//the acceleration = F/m, and F = coefficient of friction (1) * weight (mg), so a = mg/m = g
		double maxJerk = Robot.getConst("MP_MaxJerk", 4);
			//Kevin said that a good random small number was 4
		Trajectory.Config config = new Trajectory.Config(fit, samples, timestep, maxVelocity, maxAccel, maxJerk);
		traj = Pathfinder.generate(pt, config);
		i = 0;
		//TODO Use the built-in modifications to make everything better
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Trajectory.Segment seg = traj.get(i);
		double prevX = i == 0 ? 0 : traj.get(i-1).x;
		double prevY = i == 0 ? 0 : traj.get(i-1).y;
		double[] newPoint = new double[2];
		newPoint[0] = seg.x - prevX;
		newPoint[1] = seg.y - prevY;
		Scheduler.getInstance().add(new PIDMove(newPoint, dt, sd, avg));
		i++;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return i >= traj.length();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private int findSamples(double key) {
		switch ((int)key) {
		case 0:
			return Trajectory.Config.SAMPLES_FAST;
		case 1:
			return Trajectory.Config.SAMPLES_LOW;
		case 2:
			return Trajectory.Config.SAMPLES_HIGH;
		default:
			return Trajectory.Config.SAMPLES_FAST;
		}
	}
}
