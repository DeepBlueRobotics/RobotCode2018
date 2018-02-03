package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;

//import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMoveTo extends CommandGroup {
	
    public AutoMoveTo(String[] args) {
        //requires(Drivetrain);
    	double rotation;
    	double[] point;
    	for (String arg : args) {
    		if (AutoUtils.isDouble(arg)) {
    			rotation = Double.valueOf(arg);
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
