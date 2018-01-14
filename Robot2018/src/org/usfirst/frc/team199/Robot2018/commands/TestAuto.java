package org.usfirst.frc.team199.Robot2018.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Used for testing the autonomous interfaces. Runs the script specified in
 * SmartDashboard
 */
public class TestAuto extends CommandGroup implements TestAutoInterface {

    public TestAuto() {
    		// TODO
    	
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

	@Override
	/**
	 * {@inheritDoc}
	 */
	public String getScriptToTest() {
		// TODO 
		return null;
	}
}
