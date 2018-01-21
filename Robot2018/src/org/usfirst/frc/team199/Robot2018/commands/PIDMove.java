package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDMove extends Command {

	double target;
	DrivetrainInterface dt;

	public PIDMove(double targ, DrivetrainInterface dt) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		target = targ;
		this.dt = dt;
		requires(Robot.dt);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		dt.resetEnc();
		dt.enableMovePid();
		dt.setMoveSetpoint(target);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		dt.arcadeDrive(dt.getPidOut(), 0);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return dt.onDriveTarg();
	}

	// Called once after isFinished returns true
	protected void end() {
		dt.disableMovePid();
		dt.stopDrive();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
