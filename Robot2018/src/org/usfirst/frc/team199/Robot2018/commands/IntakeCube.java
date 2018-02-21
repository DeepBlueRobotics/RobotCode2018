package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeCube extends Command {

	private Timer tim;
	private boolean overDraw;

	public IntakeCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		tim = new Timer();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		tim.reset();
		tim.start();
		overDraw = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.intakeEject.runIntake(-1);
		if (Robot.intakeEject.hasCube()) {
			if (!overDraw) {
				overDraw = true;
				tim.start();
			}
		} else {
			overDraw = false;
			tim.stop();
			tim.reset();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return tim.get() > Robot.getConst("Has Cube Timeout", 0.5);
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
