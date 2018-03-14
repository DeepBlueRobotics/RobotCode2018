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

			desiredDist = lift.getDesiredDistFromPos(desiredPos);
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
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
