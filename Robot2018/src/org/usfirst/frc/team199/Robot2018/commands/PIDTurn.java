package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
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
	private PIDController turnController;
	private PIDSource ahrs;
	private Timer tim;
	private double lastTime;
	private SmartDashboardInterface sd;
	private double[] point;
	private boolean turnToPoint;
	private boolean absoluteRotation;

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the AHRS (gyro) object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param target
	 *            the target bearing (in degrees) to turn to (so negative if turning
	 *            left, positive if turning right)
	 * @param point
	 *            the target point (in inches) to turn to, relative to the starting
	 *            position
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param ahrs
	 *            the AHRS (gyro)
	 * @param sd
	 *            the Smart Dashboard reference, or a SmartDashboardInterface for
	 *            testing
	 * @param absoluteRotation
	 *            whether the target passed is absolute or relative
	 * @param turnToPoint
	 *            whether to use the point or the target for turning
	 */
	public PIDTurn(double target, double[] point, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource ahrs,
			boolean absoluteRotation, boolean turnToPoint) {
		this.target = target;
		System.out.println(this.target);
		this.point = point;
		this.dt = dt;
		this.ahrs = ahrs;
		this.sd = sd;
		this.turnToPoint = turnToPoint;
		this.absoluteRotation = absoluteRotation;

		if (Robot.dt != null) {
			requires(Robot.dt);
		}
		turnController = new PIDController(0, 0, 0, 0, ahrs, this);
		// tim = new Timer();
		sd.putData("Turn PID", turnController);
	}

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the AHRS (gyro) object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param target
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
	 * @param absoluteRotation
	 *            whether the target passed is absolute or relative
	 */
	public PIDTurn(double target, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource ahrs,
			boolean absoluteRotation) {
		this(target, null, dt, sd, ahrs, absoluteRotation, false);
	}

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the AHRS (gyro) object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param target
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
	public PIDTurn(double target, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource ahrs) {
		this(target, dt, sd, ahrs, false);
	}

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the AHRS (gyro) object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param point
	 *            the target point (in inches) to turn to, relative to the starting
	 *            position
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param ahrs
	 *            the AHRS (gyro)
	 * @param sd
	 *            the Smart Dashboard reference, or a SmartDashboardInterface for
	 *            testing
	 */
	public PIDTurn(double[] point, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource ahrs) {
		this(0, point, dt, sd, ahrs, true, true);
	}

	/**
	 * Called just before this Command runs the first time. Resets the gyro, sets
	 * the turnController's settings, and then enables it.
	 */
	@Override
	protected void initialize() {
		// calculate pid constants

		// max turn speed from FindTurnTimeConstant, converted to degrees
		double maxTurnSpeed = Robot.getConst("Max Turn Radians Per Second", 4.0) * 180 / Math.PI;
		double updateTime = sd.getConst("Default PID Update Time", 0.05);
		double r = Robot.getConst("TurnPidR", 3.0);

		double kP = r / Robot.getConst("TurnTimeConstant", 0.2) / maxTurnSpeed;
		double kI = 0;
		double kD = r / (maxTurnSpeed * updateTime);
		double kF = 1 / maxTurnSpeed * updateTime;
		turnController.setPID(kP, kI, kD, kF);

		if (!turnToPoint) {
			if (absoluteRotation) {
				target -= AutoUtils.state.getRot();
			}
		} else {
			double dx = point[0] - AutoUtils.state.getX();
			double dy = point[1] - AutoUtils.state.getY();

			System.out.println("x = " + dx + ", y = " + dy);

			// x and y are switched because we are using bearings
			double absTurn = Math.toDegrees(Math.atan2(dx, dy));
			target = absTurn - AutoUtils.state.getRot();
			System.out.println("current bearing = " + AutoUtils.state.getRot());
			System.out.println("target bearing = " + target);
		}

		System.out.println("Turn to point: " + turnToPoint);

		turnController.disable();
		if (dt.isVPIDUsed()) {
			dt.enableVelocityPIDs();
		}
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
		double newSetPoint = target + dt.getAHRSAngle();
		while (Math.abs(newSetPoint) > 180) {
			newSetPoint = newSetPoint - Math.signum(newSetPoint) * 360;
		}
		System.out.println("set point = " + newSetPoint);
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
		sd.putNumber("Turn PID Result", turnController.get());
		sd.putNumber("Turn PID Error", turnController.getError());
		// turnController.free();

		AutoUtils.state.setRot(dt.getAHRSAngle());

		dt.disableVelocityPIDs();
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
	 * 
	 * @return that distance
	 */
	private double getDistanceBetweenWheels() {
		return sd.getConst("Distance Between Wheels", 26.25);
	}
}