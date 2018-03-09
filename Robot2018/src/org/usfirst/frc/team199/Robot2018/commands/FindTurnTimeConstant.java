package org.usfirst.frc.team199.Robot2018.commands;

import java.util.ArrayList;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Do an experiment to calculate the time constant for PIDTurn, and place value
 * in preferences as "TurnTimeConstant". Only runs if we're in testing mode.
 */
public class FindTurnTimeConstant extends Command {
	private DrivetrainInterface dt;
	private SmartDashboardInterface sd;
	private AHRS gyro;
	private Robot robot;
	private Timer tim = new Timer();
	private ArrayList<DataPoint> dataPoints = new ArrayList<DataPoint>();

	private class DataPoint {
		public double seconds;
		public double degreesPerSecond;

		public DataPoint(double seconds, double degreesPerSecond) {
			this.seconds = seconds;
			this.degreesPerSecond = degreesPerSecond;
		}

		public String toString() {
			return "Time: " + seconds + " seconds; rotational velocity: " + degreesPerSecond + " degrees per second";
		}
	}

	/**
	 * Constructs this command.
	 */
	public FindTurnTimeConstant(Robot robot, DrivetrainInterface dt, AHRS gyro, SmartDashboardInterface sd) {
		if (Robot.dt != null) {
			requires(Robot.dt);
		}
		this.robot = robot;
		this.dt = dt;
		this.gyro = gyro;
		this.sd = sd;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("FindTurnTimeConstant is in init()");
		if (!robot.isTest()) {
			System.out.println("but you're not in test mode.");
			return;
		}

		System.out.println("and you're in test mode");
		tim.start();
		// spin at full speed
		dt.arcadeDrive(0, 1);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// add a new data point with the current time and rotational velocity
		DataPoint newMeasurement = new DataPoint(tim.get(), gyro.getRate());
		System.out.println(newMeasurement);
		dataPoints.add(newMeasurement);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		int numberOfDataPoints = dataPoints.size();
		// true if the last and second-to-last rotational velocities measured are within
		// 5 degrees per second squared of each other. or if it's not a test, finish
		// immediately
		if (numberOfDataPoints < 2) {
			return false;
		}
		return (Math.abs(dataPoints.get(numberOfDataPoints - 1).degreesPerSecond
				- dataPoints.get(numberOfDataPoints - 2).degreesPerSecond) < 5) || !robot.isTest();
	}

	// Called once after isFinished returns true
	protected void end() {
		// stop moving
		dt.arcadeDrive(0, 0);

		DataPoint lastDataPoint = dataPoints.get(dataPoints.size() - 1);
		System.out.println("Finished when " + lastDataPoint);
		// the rotvel of the data point we're looking for to find the time constant
		double magicValue = lastDataPoint.degreesPerSecond * (1 - (1 / Math.E));

		for (DataPoint datum : dataPoints) {
			// if we've past the magic value, we're done
			if (Math.abs(datum.degreesPerSecond) > Math.abs(magicValue)) {
				System.out.println("The magic value is " + datum);
				Robot.putConst("TurnTimeConstant", datum.seconds);
				// stop checking data
				break;
			}
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
