package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
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
	 */
	public PIDTurn(double targ, DrivetrainInterface dt, AHRS ahrs) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.dt);
		target = targ;
		this.dt = dt;
		// calculates the maximum turning speed in degrees/sec based on the max linear
		// speed in inches/s and the distance (inches) between sides of the DT
		double maxTurnSpeed = dt.getCurrentMaxSpeed() * 360
				/ (Math.PI * Robot.getConst("Distance Between Wheels", 26.25));
		double kf = 1 / (maxTurnSpeed * Robot.getConst("Default PID Update Time", 0.05));
		turnController = new PIDController(Robot.getConst("TurnkP", 1), Robot.getConst("TurnkI", 0),
				Robot.getConst("TurnkD", 0), kf, ahrs, this);
	}

	/**
	 * Called just before this Command runs the first time. Resets the gyro, sets
	 * the turnController's settings, and then enables it.
	 */
	@Override
	protected void initialize() {
		dt.resetAHRS();
		turnController.disable();
		// input is in degrees
		turnController.setInputRange(-180, 180);
		// output in "motor units" (arcade and tank only accept values [-1, 1]
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setContinuous();
		turnController.setAbsoluteTolerance(Robot.getConst("TurnTolerance", 1));
		turnController.setSetpoint(target);
		SmartDashboard.putData("Turn PID", turnController);
		turnController.enable();
		dt.enableVelocityPIDs();
	}

	/**
	 * Called repeatedly when this Command is scheduled to run. This method is empty
	 * bc the turnController runs on a different thread as soon as it is enabled in
	 * initialize().
	 */
	@Override
	protected void execute() {
		SmartDashboard.putNumber("Turn PID Result", turnController.get());
		SmartDashboard.putNumber("Turn PID Error", turnController.getError());
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
		return turnController.onTarget();
	}

	/**
	 * Called once after isFinished returns true. Disables and frees the
	 * turnController, essentially turning it "off" and "deleting" the thread it was
	 * running on.
	 */
	@Override
	protected void end() {
		turnController.disable();
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
	}
}
