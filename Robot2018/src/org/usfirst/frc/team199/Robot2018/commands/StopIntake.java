package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class StopIntake extends InstantCommand {

	public StopIntake() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.intakeEject.stopIntake();
	}
}
