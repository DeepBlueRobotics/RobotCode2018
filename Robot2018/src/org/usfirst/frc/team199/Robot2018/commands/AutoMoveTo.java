package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.PIDSource;

//import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * The AutoMoveTo command, which makes the robot go through a set of movements
 * and rotations.
 */
public class AutoMoveTo extends CommandGroup {

	public AutoMoveTo(String[] args, double centerOfRot, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource pidMoveSrc,
			PIDSource pidTurnSource) {
		// requires(Drivetrain);
		double rotation;
		double[] point = { 0, 0 };
		
		for (int i = 0; i < args.length - 1; i++) {
			point = AutoUtils.parsePoint(args[i]);
			addSequential(new PIDTurn(point, dt, sd, pidTurnSource));
			
			double dist = Math.sqrt(point[0]*point[0] + point[1]*point[1]) + centerOfRot; // TODO: subtract rotation error
			addSequential(new PIDMove(dist, dt, sd, pidMoveSrc));
		}
		
		String lastArg = args[args.length - 1];
		// if the last argument is a point, no need for extra positioning at the end
		if (AutoUtils.isPoint(lastArg)) { 
			point = AutoUtils.parsePoint(lastArg);
			addSequential(new PIDTurn(point, dt, sd, pidTurnSource));
			addSequential(new PIDMove(point, dt, sd, pidMoveSrc));
		} else if (AutoUtils.isDouble(lastArg)) { // move a little after turn to end up in the correct position
			rotation = Double.valueOf(lastArg);
			addSequential(new PIDTurn(rotation, dt, sd, pidTurnSource, true));
			addSequential(new PIDTurn(centerOfRot, dt, sd, pidTurnSource));
		} else {
			throw new IllegalArgumentException();
		}
	}

	public AutoMoveTo(String[] args, double centerOfRot) {
		this(args, centerOfRot, Robot.dt, Robot.sd, Robot.dt.getDistEncAvg(), Robot.dt.getGyro());
	}
}
