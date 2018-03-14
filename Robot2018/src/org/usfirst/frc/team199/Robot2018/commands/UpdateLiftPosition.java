package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.LiftHeight;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command should be the default command for the Lift subsystem
 */
public class UpdateLiftPosition extends Command {

	private Lift lift;

	private double currDist;
	private double desiredDist = 0;
	private LiftHeight desiredPos;

	private boolean manipulatorPluggedIn;

	public UpdateLiftPosition(Lift lift) {
		requires(Robot.lift);
		this.lift = lift;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		try {
			Robot.oi.manipulator.getRawAxis(1);
		} catch (NullPointerException e) {
			System.err.println("[ERROR] Manipulator not plugged in.");
			manipulatorPluggedIn = false;
		}

		// I put this enable in here as well as in the Lift constructor bc not sure if
		// this interrupted() method (and therefore end() and disable() methods) will be
		// called when an auto command takes overs temporarily. May be repetitive, so...
		/**
		 * @TODO figure out if this needs to be here and/or in Lift constructor or not
		 */
		lift.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (manipulatorPluggedIn) {
			int angle = Robot.oi.manipulator.getPOV();

			// 180 degrees is down and -1 is "nothing," so basically default to GROUND,
			// anything else is SWITCH
			if (angle != 180 && angle != -1) {
				desiredPos = LiftHeight.SWITCH;
				// NOTE: if full lift functionality does become a thing, need to add a couple
				// more if-elses here to account for those enum values
			} else {
				desiredPos = LiftHeight.GROUND;
			}

			currDist = lift.getHeight();
			/**
			 * @TODO: AutoLift sets desiredDist to the delta dist (set - current), but
			 *        shouldn't you set the setpoint to your actual setpoint? not to the
			 *        current error? Uncomment line 70 if using delta. Or, use
			 *        setRelativeSetpoint(something here).
			 */
			desiredDist = lift.getDesiredDistFromPos(desiredPos);
			// desiredDist -= currDist;
			lift.setSetpoint(desiredDist);
		}
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
		lift.disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
