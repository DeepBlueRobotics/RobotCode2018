package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Use this command to test the Lift with PID
 */
public class MoveLiftWithPID extends Command {

	private Lift lift;
	private double setpoint;
	private int dir;

	public MoveLiftWithPID(Lift lift, boolean up) {
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
		setpoint = 0;
		lift.setSetpoint(setpoint);
		lift.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		setpoint += dir * Robot.getConst("Lift Move Increment", 1); // inches
		lift.setSetpoint(setpoint);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// always false bc whileHeld
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		lift.disable();
		// keep stopLift() after disable() so they don't conflict
		lift.stopLift();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
