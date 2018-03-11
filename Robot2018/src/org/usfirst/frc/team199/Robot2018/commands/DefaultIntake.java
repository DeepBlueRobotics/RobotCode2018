package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEjectInterface;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DefaultIntake extends Command {

	private boolean manipulatorPluggedIn = true;
	private IntakeEjectInterface intEj;

	public DefaultIntake(IntakeEjectInterface intEj) {
		requires(Robot.intakeEject);
		this.intEj = intEj;
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
		// 1 and 5 represent the axes' index in driver station
		if (manipulatorPluggedIn) {
			intEj.runLeftIntake(Robot.oi.manipulator.getRawAxis(1));
			intEj.runRightIntake(Robot.oi.manipulator.getRawAxis(5));
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
		intEj.runIntake(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
