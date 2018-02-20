package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turns the robot to a certain target bearing using PID. Implements PIDOutput
 * in order to keep turn PID centralized in this command.
 */
public class PIDTurn extends Command implements PIDOutput {

	private double target;
	private DrivetrainInterface dt;
	private SmartDashboardInterface sd;
	private PIDController turnController;
	private PIDSource ahrs;
	private Timer tim;
	private double lastTime;

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the AHRS (gyro) object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param targ
	 *            the target bearing (in degrees) to turn to (so negative if turning
	 *            left, positive if turning right)
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param ahrs
	 *            the AHRS (gyro)
	 * @param sd
	 *            the Smart Dashboard reference, or a SmartDashboardInterface for
	 *            testing
	 */
	public PIDTurn(double targ, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource ahrs) {
		// Use requires() here to declare subsystem dependencies
		target = targ;
		this.dt = dt;
		this.sd = sd;
		this.ahrs = ahrs;

		if (Robot.dt != null) {
			requires(Robot.dt);
		}
		// calculates the maximum turning speed in degrees/sec based on the max linear
		// speed in inches/s and the distance (inches) between sides of the DT
		double maxTurnSpeed = dt.getCurrentMaxSpeed() * 360 / (Math.PI * getDistanceBetweenWheels());
		double kf = 1 / (maxTurnSpeed * sd.getConst("Default PID Update Time", 0.05));
		turnController = new PIDController(sd.getConst("TurnkP", 1), sd.getConst("TurnkI", 0), sd.getConst("TurnkD", 0),
				kf, ahrs, this) {
			/**
			 * Turn Velocity: V = 4r sqrt((T*G*theta) / (R*m))
			 * where r = half of distance between wheels
			 * T = max torque of wheels
			 * G = gear ratio
			 * theta = rotational distance to end of turn
			 * R = radius of wheels
			 * m = mass
			 */
			@Override
			protected double calculateFeedForward() {
				double feedForwardConst = dt.getPIDTurnConstant();
				return feedForwardConst * (getDistanceBetweenWheels()/2) * (targ / Math.abs(targ)) * Math.sqrt(Math.abs(targ));
			}
		};
		// tim = new Timer();
		SmartDashboard.putData("Turn PID", turnController);
	}

	/**
	 * Called just before this Command runs the first time. Resets the gyro, sets
	 * the turnController's settings, and then enables it.
	 */
	@Override
	protected void initialize() {
		turnController.disable();
		// dt.enableVelocityPIDs();
		System.out.println("initialize2s");
		// dt.resetAHRS();
		System.out.println("after reset");
		System.out.println("after disabling");
		// input is in degrees
		turnController.setInputRange(-180, 180);
		// output in "motor units" (arcade and tank only accept values [-1, 1]
		turnController.setOutputRange(Robot.getConst("Output", 0.5) * -1, Robot.getConst("Output", 0.5));
		turnController.setContinuous(true);
		turnController.setAbsoluteTolerance(Robot.getConst("TurnTolerance", 1));
		double newSetPoint = Robot.getConst("Turn Targ", 90) + dt.getAHRSAngle();
		while (Math.abs(newSetPoint) > 180) {
			newSetPoint = newSetPoint - Math.signum(newSetPoint) * 360;
		}
		turnController.setSetpoint(newSetPoint);

		turnController.enable();
		System.out.println("initialize finished");
		// tim.start();
		// lastTime = tim.get();
	}

	/**
	 * Called repeatedly when this Command is scheduled to run. This method is empty
	 * bc the turnController runs on a different thread as soon as it is enabled in
	 * initialize().
	 */
	@Override
	protected void execute() {
		System.out.println("execute");
		System.out.println("Angle: " + dt.getAHRSAngle());
		// if (tim.get() > lastTime + Robot.getConst("Update Time", 1)) {
		// SmartDashboard.putNumber("Angle", dt.getAHRSAngle());
		// lastTime = tim.get();
		// }
	}

	/**
	 * Tells this command to terminate when the turnController has reached its
	 * target.
	 * 
	 * @return true if the robot has turned to the correct target bearing, false if
	 *         not
	 */
	@Override
	protected boolean isFinished() {
		System.out.println("isFinished");
		return (turnController.onTarget() && Math.abs(dt.getGyroRate()) < 1);
		// return turnController.onTarget()
		// && Math.abs(dt.getLeftEncRate()) <= Robot.getConst("Maximum Velocity When
		// Stop", 1)
		// && Math.abs(dt.getRightEncRate()) <= Robot.getConst("Maximum Velocity When
		// Stop", 1);
	}

	/**
	 * Called once after isFinished returns true. Disables and frees the
	 * turnController, essentially turning it "off" and "deleting" the thread it was
	 * running on.
	 */
	@Override
	protected void end() {
		turnController.disable();
		System.out.println("end");
		SmartDashboard.putNumber("Turn PID Result", turnController.get());
		SmartDashboard.putNumber("Turn PID Error", turnController.getError());
		// turnController.free();
	}

	/**
	 * Called when another command which requires one or more of the same subsystems
	 * is scheduled to run. Disables and frees the turnController, essentially
	 * turning it "off" and "deleting" the thread it was running on.
	 */
	@Override
	protected void interrupted() {
		end();
	}

	/**
	 * Implementation of PIDOutput's pidWrite method. Sends the turnController's
	 * output speed to the motors as the turn speed in DifferentialDrive's
	 * arcadeDrive method.
	 * 
	 * @param output
	 *            the output turn speed [-1, 1], calculated by the turn
	 *            PIDController, to pass in as the turn speed in arcadeDrive()
	 */
	@Override
	public void pidWrite(double output) {
		dt.arcadeDrive(0, output);
		SmartDashboard.putNumber("Turn PID Output", output);
	}
	
	/**
	 * Gets the distance between the two middle wheels.
	 * @return that distance
	 */
	private double getDistanceBetweenWheels() {
		return sd.getConst("Distance Between Wheels", 26.25);
	}
}
