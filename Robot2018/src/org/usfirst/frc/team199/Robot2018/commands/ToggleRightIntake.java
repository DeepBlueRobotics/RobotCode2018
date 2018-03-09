package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ToggleRightIntake extends InstantCommand {

	public ToggleRightIntake() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intakeEject);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.intakeEject.toggleRightIntake();
	}

}
