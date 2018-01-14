package org.usfirst.frc.team199.Robot2018.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Initially run during Auto. Responsible for getting input from SmartDashboard
 * and the FMS, and then calling RunAuto with the specified script.  
 */
public class Autonomous extends CommandGroup implements AutonomousInterface {

    public Autonomous() {
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

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Position getStartingPos() {
		// TODO 
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Strategy[] getStrategies() {
		// TODO 
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double getDelay() {
		// TODO 
		return 0;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String pickScript() {
		// TODO 
		return null;
	}
}
