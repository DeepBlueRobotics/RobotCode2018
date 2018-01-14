package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDTurn extends Command {

	double target;
    public PIDTurn(double targ) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	target = targ;
    	requires(Robot.dt);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.resetAHRS();
    	Robot.dt.setSetTurn(target);
    	Robot.dt.enableTurnPid();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.arcadeDrive(0, Robot.dt.getPidOut());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.dt.onTurnTarg();
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
