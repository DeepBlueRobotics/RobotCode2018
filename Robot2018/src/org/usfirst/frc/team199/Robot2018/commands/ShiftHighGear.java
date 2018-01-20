package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShiftHighGear extends Command {

	Timer tim;

	public ShiftHighGear() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		tim = new Timer();
		tim.reset();
		tim.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.dt.changeShiftGear(true);
		SmartDashboard.putBoolean("High Gear", true);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return tim.get() > Robot.getConst("Piston Act Time", 0.1);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.dt.turnGearSolenoidOff();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
