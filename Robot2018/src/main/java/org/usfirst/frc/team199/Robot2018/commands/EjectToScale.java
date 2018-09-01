package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class EjectToScale extends CommandGroup {

	public EjectToScale(double dist) {
		addSequential(new AutoLift(Robot.lift, "SCALE"));
		addSequential(new PIDMove(dist, Robot.dt, Robot.sd, Robot.dt.getDistEncAvg()));
		addSequential(new OuttakeCube());
		addSequential(new PIDMove(-1 * Robot.getConst("Scale move back dist", 12), Robot.dt, Robot.sd,
				Robot.dt.getDistEncAvg()));
		addSequential(new AutoLift(Robot.lift, "GROUND"));
	}
}
