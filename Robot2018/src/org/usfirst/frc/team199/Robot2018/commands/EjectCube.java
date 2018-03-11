package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEjectInterface;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class EjectCube extends Command {

	private Timer tim;
	private IntakeEjectInterface intEj;

	public EjectCube(IntakeEjectInterface intEj) {
		requires(Robot.intakeEject);
		this.intEj = intEj;
		tim = new Timer();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		tim.reset();
		tim.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		intEj.runIntake(1);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return tim.get() > Robot.getConst("Outake Time", 0.5);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		intEj.runIntake(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
