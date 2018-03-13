package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.LiftHeight;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftToPosition extends Command {

	private Lift lift;
	private LiftHeight pos;

	public LiftToPosition(Lift lift, String goal) {
		requires(Robot.lift);
		this.lift = lift;
		pos = LiftHeight.toLH(goal);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// There's no way to reset the Lift's PID constants after constructing it, so if
		// you change them you must Restart Robot Code in DS.
		double setpoint = lift.getDesiredDistFromPos(pos);
		lift.setSetpoint(setpoint);
		System.out.println("Target Height: " + setpoint);
		lift.enable();
		System.out.println("Enabled");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println("Current Height: " + lift.getHeight());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return lift.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		lift.disable();
		System.out.println("Disabled");
		// keep stopLift() after disable() so they don't conflict
		lift.stopLift();
		System.out.println("Stopped");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
