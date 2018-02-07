package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.Position;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoLift extends Command implements PIDOutput{
	
	// All distances are - 1 foot for initial height of intake and + 3 inches for wiggle room for dropping cubes
	//Distance to switch
	// 18.75 inches in starting position
	private final double SWITCH_DIST = 9.75;
	//Distance to scale
	// 5 feet starting
	private final double SCALE_DIST = 51;
	//Distance to bar
	// 7 feet starting; bar distance should be changed because I'm not aware how climber mech will be positioned
	private final double BAR_DIST = 72;
	private double desiredDist = 0;
	private double currDist = 0;
	private LiftInterface lift;
	private Position desiredPos;
	
	private PIDController liftController;

    public AutoLift(Position stage, LiftInterface lift) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.lift = lift;
    	requires(Robot.lift);
    	switch(lift.getCurrPos()) {
    	case GROUND:
    		currDist = 0;
    		break;
    	case SWITCH:
    		currDist = SWITCH_DIST;
    		break;
    	case SCALE:
    		currDist = SCALE_DIST;
    		break;
    	case BAR:
    		currDist = BAR_DIST;
    		break;
    	}
    	switch(stage) {
    	case GROUND:
    		desiredDist = -currDist;
    		break;
    	case SWITCH:
    		desiredDist = SWITCH_DIST - currDist;
    		break;
    	case SCALE:
    		desiredDist = SCALE_DIST - currDist;
    		break;
    	case BAR:
    		desiredDist = BAR_DIST - currDist;
    		break;
    	}
    	
    	liftController = new PIDController(Robot.getConst("LiftkP", 1), Robot.getConst("LiftkI", 0),
				Robot.getConst("LiftkD", 0),RobotMap.liftEnc, this);
    	
    	desiredPos = stage;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	lift.resetEnc();
		// input is in inches
		//liftController.setInputRange(-Robot.getConst("Max High Speed", 204), Robot.getConst("Max High Speed", 204));
		// output in "motor units" (arcade and tank only accept values [-1, 1]
		liftController.setOutputRange(-1.0, 1.0);
		liftController.setContinuous(false);
		//liftController.setAbsoluteTolerance(Robot.getConst("MoveTolerance", 2));
		liftController.setSetpoint(desiredDist);
		liftController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(liftController.onTarget()) {
    		lift.setTargetPosition(desiredPos);
    		return true;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	liftController.disable();
    	liftController.free();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    
    @Override
	public void pidWrite(double output) {
		lift.runMotor(output);
	}
    
}
