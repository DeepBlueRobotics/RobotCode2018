package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class EjectToSwitch extends CommandGroup {

	public EjectToSwitch() {
		addSequential(new AutoLift(Robot.lift, "SWITCH"));
		addSequential(
				new PIDMove(Robot.getConst("Auto Switch Move Dist", 12), Robot.dt, Robot.sd, Robot.dt.getDistEncAvg()));
		addSequential(new OuttakeCube());
		addSequential(new PIDMove(-1 * Robot.getConst("Auto Switch Move Dist", 12), Robot.dt, Robot.sd,
				Robot.dt.getDistEncAvg()));
		addSequential(new AutoLift(Robot.lift, "GROUND"));
	}
}
