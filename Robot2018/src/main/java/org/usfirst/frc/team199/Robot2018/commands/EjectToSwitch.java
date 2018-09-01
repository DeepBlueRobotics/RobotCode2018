package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class EjectToSwitch extends CommandGroup {

	public EjectToSwitch(double dist) {
//		addSequential(new AutoLift(Robot.lift, "SWITCH"));
		addSequential(new PIDMove(dist, Robot.dt, Robot.sd, Robot.dt.getDistEncAvg()));
		addSequential(new OuttakeCube());
		addSequential(new PIDMove(-1 * Robot.getConst("Switch move back dist", 12), Robot.dt, Robot.sd,
				Robot.dt.getDistEncAvg()));
		addParallel(new AutoLift(Robot.lift, "GROUND"));
	}
}
