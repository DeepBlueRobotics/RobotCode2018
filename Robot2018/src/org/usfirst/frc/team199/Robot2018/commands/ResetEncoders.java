package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Resets left and right drivetrain encoders to 0
 */
public class ResetEncoders extends InstantCommand {

	public ResetEncoders() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.dt.resetDistEncs();
	}
}
