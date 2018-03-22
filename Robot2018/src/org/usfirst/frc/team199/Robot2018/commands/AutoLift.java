package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.subsystems.Lift;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.LiftHeight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLift extends CommandGroup {

	public AutoLift(Lift lift, String height) {
		if (height.equals("GROUND")) {
			addSequential(new LiftToPosition(lift, LiftHeight.toLH("HOLD_CUBE")));
		}
		addSequential(new LiftToPosition(lift, LiftHeight.toLH(height)));
	}
}
