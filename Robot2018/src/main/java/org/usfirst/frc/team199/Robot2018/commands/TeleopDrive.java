/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command. You can replace me with your own command.
 */
public class TeleopDrive extends Command {
	public TeleopDrive() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.dt);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (Robot.getBool("Teleop velocity PID", false)) {
			Robot.dt.enableVelocityPIDs();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Robot.getBool("Teleop velocity PID", false)) {
			Robot.dt.resetVPIDAndRobotDriveRanges();
			Robot.dt.resetAllVelocityPIDConsts();
			if (Robot.dt.VPIDsOnTarg()) {
				Robot.dt.disableVelocityPIDs();
			} else {
				Robot.dt.enableVelocityPIDs();
			}
		}
		Robot.dt.teleopDrive();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.dt.disableVelocityPIDs();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
