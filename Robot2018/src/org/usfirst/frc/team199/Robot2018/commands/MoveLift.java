package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Use this command to test the Lift without PID
 */
public class MoveLift extends Command {

	private Lift lift;
	private double SPEED;
	private int dir;

	public MoveLift(Lift lift, boolean up) {
		requires(Robot.lift);
		this.lift = lift;
		if (up)
			dir = 1;
		else
			dir = -1;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		lift.disable();
		// below Lift Speed may be for manual testing only, but commented out code 2
		// lines below can be used if want max speed
		SPEED = Robot.getConst("Lift Speed", 0.5);
		// SPEED = lift.getLiftMaxSpeed();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		lift.runMotor(SPEED * dir);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		lift.stopLift();
		lift.enable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
