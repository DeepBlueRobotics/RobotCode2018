package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DefaultIntake extends Command {

	private boolean manipulatorPluggedIn = true;

	public DefaultIntake() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intakeEject);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// try {
		// Robot.oi.manipulator.getRawAxis(1);
		// } catch (NullPointerException e) {
		// System.err.println("[ERROR] Manipulator not plugged in.");
		// manipulatorPluggedIn = false;
		// }
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// 1 and 5 represent the axes' index in driver station
		// if (manipulatorPluggedIn) {
		// Robot.intakeEject.runLeftIntake(Robot.oi.manipulator.getRawAxis(1));
		// Robot.intakeEject.runRightIntake(Robot.oi.manipulator.getRawAxis(5));
		// }
		Robot.intakeEject.runLeftIntake(Robot.getConst("Intake Holding Speed", 0.15));
		Robot.intakeEject.runRightIntake(Robot.getConst("Intake Holding Speed", 0.15));

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.intakeEject.runIntake(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
