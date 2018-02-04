package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetDistancePerPulse extends InstantCommand {

	public SetDistancePerPulse() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.dt.setDistancePerPulseLeft(Robot.getConst("Distance Per Pulse Left", 0.184));
		Robot.dt.setDistancePerPulseRight(Robot.getConst("Distance Per Pulse Right", 0.184));
	}
}
