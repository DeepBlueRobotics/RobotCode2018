package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command should be the default command for the Lift subsystem
 */
public class UpdateLiftPosition extends Command {

	private Lift lift;
	private double targ;

	public UpdateLiftPosition(Lift lift) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.lift = lift;
		targ = 0;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		/**
		 * May need to setInputRange(0, maxHeight);
		 */
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		int angle = Robot.oi.manipulator.getPOV();
		/**
		 * reference AutoLift for how to calculate targ (below) based on the given
		 * Position enum
		 */
		if (angle == 0) {
			// targ = GROUND
		} else {
			// targ = SWITCH
		}
		lift.setSetpoint(targ);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		lift.disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
