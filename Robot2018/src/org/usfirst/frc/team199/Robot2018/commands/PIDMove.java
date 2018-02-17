package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives the robot a certain target distance using PID. Implements PIDOutput in
 * order to keep move PID centralized in this command.
 */
public class PIDMove extends Command implements PIDOutput {

	private double target;
	private DrivetrainInterface dt;
	private PIDController moveController;
	private PIDSourceAverage avg;

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the encoder average object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param targ
	 *            the target distance (in inches) to move to
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param avg
	 *            the PIDSourceAverage of the DT's two Encoders
	 */
	public PIDMove(double targ, DrivetrainInterface dt, PIDSourceAverage avg) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.dt);
		target = targ;
		this.dt = dt;
		this.avg = avg;
		double kf = 1 / (dt.getCurrentMaxSpeed() * Robot.getConst("Default PID Update Time", 0.05));
		moveController = new PIDController(Robot.getConst("MovekP", 0), Robot.getConst("MovekI", 0),
				Robot.getConst("MovekD", 0), kf, avg, this);
	}

	/**
	 * Called just before this Command runs the first time. Resets the distance
	 * encoders, sets the moveController's settings, and then enables it.
	 */
	@Override
	public void initialize() {
		dt.resetDistEncs();
		moveController.disable();
		// input is in inches
		moveController.setInputRange(-Robot.getConst("Max High Speed", 204), Robot.getConst("Max High Speed", 204));
		// output in "motor units" (arcade and tank only accept values [-1, 1]
		moveController.setOutputRange(-1.0, 1.0);
		moveController.setContinuous(false);
		moveController.setAbsoluteTolerance(Robot.getConst("MoveTolerance", 0.1));
		moveController.setSetpoint(Robot.getConst("Move Targ", 24));

		SmartDashboard.putData("Move PID", moveController);

		moveController.enable();
		// dt.enableVelocityPIDs();
	}

	/**
	 * Called repeatedly when this Command is scheduled to run. This method is empty
	 * bc the moveController runs on a different thread as soon as it is enabled in
	 * initialize().
	 */
	@Override
	protected void execute() {
		System.out.println("Enc Avg Dist: " + avg.pidGet());
		SmartDashboard.putNumber("Move PID Result", moveController.get());
		SmartDashboard.putNumber("Move PID Error", moveController.getError());
	}

	/**
	 * Tells this command to terminate when the moveController has reached its
	 * target.
	 * 
	 * @return true if the robot has moved the correct distance, false if not
	 */
	@Override
	protected boolean isFinished() {
		return moveController.onTarget();
	}

	/**
	 * Called once after isFinished returns true. Disables and frees the
	 * moveController, essentially turning it "off" and "deleting" the thread it was
	 * running on.
	 */
	@Override
	protected void end() {
		moveController.disable();
		System.out.println("End");
		// moveController.free();
	}

	/**
	 * Called when another command which requires one or more of the same subsystems
	 * is scheduled to run. Disables and frees the moveController, essentially
	 * turning it "off" and "deleting" the thread it was running on.
	 */
	@Override
	protected void interrupted() {
		end();
	}

	/**
	 * Implementation of PIDOutput's pidWrite method. Sends the moveController's
	 * output speed to the motors as the drive/forward speed in DifferentialDrive's
	 * arcadeDrive method.
	 * 
	 * @param output
	 *            the output drive/forward speed [-1, 1], calculated by the move
	 *            PIDController, to pass in as the drive/forward speed in
	 *            arcadeDrive()
	 */
	@Override
	public void pidWrite(double output) {
		dt.arcadeDrive(output, 0);
		SmartDashboard.putNumber("Move PID Output", output);
	}
}
