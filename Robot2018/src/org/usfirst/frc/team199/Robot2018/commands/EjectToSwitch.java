package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEjectInterface;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class EjectToSwitch extends Command {

	private IntakeEjectInterface intEj;

	public EjectToSwitch(IntakeEjectInterface intEj) {
		requires(Robot.intakeEject);
		this.intEj = intEj;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
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
	}
}
