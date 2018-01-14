/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.robot.commands;

import org.usfirst.frc.team199.robot.Robot;
import org.usfirst.frc.team199.robot.motion.PID;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class TeleopDrive extends Command {
	public TeleopDrive() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.dt);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(SmartDashboard.getBoolean("Arcade Drive", true)) {
			if(SmartDashboard.getBoolean("Arcade Drive Default Setup", true)) {
				Robot.dt.arcadeDrive(Robot.oi.leftJoy.getY(), Robot.oi.rightJoy.getX());
			} else {
				Robot.dt.arcadeDrive(Robot.oi.rightJoy.getY(), Robot.oi.leftJoy.getX());
			}
		} else {
			Robot.dt.tankDrive(Robot.oi.leftJoy.getY(), Robot.oi.rightJoy.getY());
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.dt.stopDrive();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
