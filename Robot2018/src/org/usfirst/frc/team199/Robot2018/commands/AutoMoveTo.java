package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;

//import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMoveTo extends CommandGroup {
	
    public AutoMoveTo(String[] args) {
        //requires(Drivetrain);
    	double rotation;
    	double[] point = new double[2];
    	String parentheseless;
    	String[] pointparts;
    	for (String arg : args) {
    		if (AutoUtils.isDouble(arg)) {
    			rotation = Double.valueOf(arg);
    			addSequential(new PIDTurn(rotation - AutoUtils.getRot(), Robot.dt, Robot.dt.getGyro()));
    			AutoUtils.setRot(rotation);
    		} else if (AutoUtils.isPoint(arg)) {
    			parentheseless = arg.substring(1, arg.length() - 1);
    			pointparts = parentheseless.split(",");
    			point[0] = Double.parseDouble(pointparts[0]);
    			point[1] = Double.parseDouble(pointparts[1]);
    			addSequential(new PIDTurn(Math.toDegrees(Math.atan((point[0] - AutoUtils.getX())/(point[1] - AutoUtils.getY())) - AutoUtils.getRot()), Robot.dt, Robot.dt.getGyro()));
    			addSequential(new PIDMove(Math.sqrt(((point[0] - AutoUtils.getX()) * (point[0] - AutoUtils.getX()) + ((point[1] - AutoUtils.getY()) * (point[1] - AutoUtils.getY())))), Robot.dt, new PIDSourceAverage(null, null)));
    			AutoUtils.setX(point[0]);
    			AutoUtils.setY(point[1]);
    			AutoUtils.setRot(Math.toDegrees(Math.atan((point[0] - AutoUtils.getX())/(point[1] - AutoUtils.getY()))));
    		} else {	
    			throw new IllegalArgumentException();
    		}
    	}
    	
    	
    	
    	// Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
