package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StopIntake extends Command {

	private Timer tim;

	public StopIntake() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super();
		// requires(Robot.intakeEject);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.stopIntake = true;
		tim = new Timer();
		tim.reset();
		tim.start();
	}

	@Override
	protected void execute() {
		// Robot.intakeEject.stopIntake();
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return tim.get() > 3;
	}
}
