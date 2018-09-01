package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Updates the PIDControllers' PIDConstants based on SmartDashboard values
 */
public class UpdatePIDConstants extends InstantCommand {

	public UpdatePIDConstants() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.dt.resetAllVelocityPIDConsts();
	}
}
