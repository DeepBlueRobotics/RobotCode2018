package org.usfirst.frc.team199.robot.commands;

import org.usfirst.frc.team199.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShiftDrive extends Command {

	Timer tim;
    public ShiftDrive() {
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
    	if(SmartDashboard.getBoolean("High Gear", false)) {
    		Robot.dt.pushGear(true);
    		SmartDashboard.putBoolean("High Gear", true);
    	} else {
    		Robot.dt.pushGear(false);
    		SmartDashboard.putBoolean("High Gear", false);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return tim.get() > 0.1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.dt.stopGear();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
