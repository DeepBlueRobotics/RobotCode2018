package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDDrive extends Command {

	double target;
    public PIDDrive(double targ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	target = targ;
    	requires(Robot.dt);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.enableMovePid();
    	Robot.dt.setSetMove(target);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.arcadeDrive(Robot.dt.getPidOut(), 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.dt.onDriveTarg();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.dt.stopDrive();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
