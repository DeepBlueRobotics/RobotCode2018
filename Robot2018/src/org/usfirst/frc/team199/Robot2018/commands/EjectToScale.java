package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class EjectToScale extends CommandGroup {

    public EjectToScale() {
    		addSequential(new LiftToPosition(Robot.lift, "SCALE"));
		addSequential(new PIDMove(Robot.getConst("Auto Scale Move Dist", 12), Robot.dt, Robot.sd, Robot.dt.getDistEncAvg()));
		addSequential(new OuttakeCube());
		addSequential(new PIDMove(-1 * Robot.getConst("Auto Scale Move Dist", 12), Robot.dt, Robot.sd, Robot.dt.getDistEncAvg()));
		addSequential(new LiftToPosition(Robot.lift, "SCALE"));
    }
}
