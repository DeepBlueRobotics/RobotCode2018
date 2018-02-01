package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Toggles between arcade and tank drive.
 */
public class ShiftDriveType extends InstantCommand {

	public ShiftDriveType() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		SmartDashboard.putBoolean("Bool/Arcade Drive", !Robot.getBool("Arcade Drive", true));
	}

}
