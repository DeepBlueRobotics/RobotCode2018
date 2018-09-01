package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OuttakeCube extends Command {

	Timer tim;

	public OuttakeCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intakeEject);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		tim = new Timer();
		tim.reset();
		tim.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.intakeEject.runIntake(1);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return tim.get() > Robot.getConst("Outake Time", 0.5);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.intakeEject.runIntake(0);
		System.out.println("Outtake done");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
