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

	public AutoMoveTo(String[] args, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource pidMoveSrc,
			PIDSource pidTurnSource) {
		// requires(Drivetrain);
		double rotation;
		double[] point = { 0, 0 };
		for (String arg : args) {
			if (AutoUtils.isDouble(arg)) {
				rotation = Double.valueOf(arg);
				addSequential(new PIDTurn(rotation, dt, sd, pidTurnSource, true));
			} else if (AutoUtils.isPoint(arg)) {
				point = AutoUtils.parsePoint(arg);
				addSequential(new PIDTurn(point, dt, sd, pidTurnSource));
				addSequential(new PIDMove(point, dt, sd, pidMoveSrc));
			} else {
				throw new IllegalArgumentException();
			}
		}
	}

	public AutoMoveTo(String[] args) {
		this(args, Robot.dt, Robot.sd, Robot.dt.getDistEncAvg(), Robot.dt.getGyro());
	}
}
