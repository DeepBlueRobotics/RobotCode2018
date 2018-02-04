package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDTurn extends Command implements PIDOutput {

	double target;
	DrivetrainInterface dt;
	private PIDController turnController;

	public PIDTurn(double targ, DrivetrainInterface dt, AHRS ahrs) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		target = targ;
		this.dt = dt;
		requires(Robot.dt);
		// calculates the maximum turning speed in degrees/sec based on the max linear
		// speed in inches/s and the distance (inches) between sides of the DT
		double maxTurnSpeed = dt.getCurrentMaxSpeed() * 360
				/ (Math.PI * Robot.getConst("Distance Between Wheels", 26.25));
		double kf = 1 / (maxTurnSpeed * Robot.getConst("Default PID Update Time", 0.05));
		turnController = new PIDController(Robot.getConst("TurnkP", 1), Robot.getConst("TurnkI", 0),
				Robot.getConst("TurnkD", 0), kf, ahrs, this);
	}

	// Called just before this Command runs the first time
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
		turnController.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// This method is empty bc the moveController runs on a different thread as soon
		// as it is enabled.
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return turnController.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		turnController.disable();
		turnController.free();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	@Override
	public void pidWrite(double output) {
		dt.arcadeDrive(0, output);
	}
}
