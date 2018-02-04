package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shifts the DT to high gear.
 */
public class ShiftHighGear extends InstantCommand {

	public ShiftHighGear() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
//		Robot.dt.shiftGears(true);
		SmartDashboard.putBoolean("High Gear", true);
		Robot.dt.resetVelocityPIDkFConsts();
	}
}
