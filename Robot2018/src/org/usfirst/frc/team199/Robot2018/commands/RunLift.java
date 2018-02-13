package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunLift extends Command {

	private LiftInterface lift;
	private final double SPEED = 0.05;
	private int dir;
	
    public RunLift(LiftInterface lift, boolean up) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.lift = lift;
        requires(Robot.lift);
    	if(up)
    		dir = 1;
    	else
    		dir = -1;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	lift.runMotor(SPEED * dir);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	lift.stopLift();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
