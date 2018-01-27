package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetDistancePerPulse extends Command {

	Timer tim;

	public SetDistancePerPulse() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		tim = new Timer();
		tim.reset();
		tim.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.dt.setDistancePerPulseLeft(Robot.getConst("Distance Per Pulse Left", 0.184));
		Robot.dt.setDistancePerPulseRight(Robot.getConst("Distance Per Pulse Right", 0.184));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return tim.get() > Robot.getConst("Update Constants Time", 0.1);
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
