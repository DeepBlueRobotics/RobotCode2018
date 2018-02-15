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

	public AutoMoveTo(String[] args, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource pidMoveSrc) {
		// requires(Drivetrain);
		double rotation;
		double[] point = { 0, 0 };
		for (String arg : args) {
			if (AutoUtils.isDouble(arg)) {
				rotation = Double.valueOf(arg);
				double relrotation = rotation - AutoUtils.position.getRot();
				addSequential(new PIDTurn(relrotation, dt, sd, pidMoveSrc));
				AutoUtils.position.setRot(rotation);
			} else if (AutoUtils.isPoint(arg)) {
				point = AutoUtils.parsePoint(arg);
				double deltaX = point[0] - AutoUtils.position.getX();
				double deltaY = point[1] - AutoUtils.position.getY();
				double atan = Math.toDegrees(Math.atan(deltaX / deltaY));
				double relrotation = atan - AutoUtils.position.getRot();
				addSequential(new PIDTurn(relrotation, dt, sd, pidMoveSrc));
				double dX2 = deltaX * deltaX;
				double dY2 = deltaY * deltaY;
				double distance = Math.sqrt(dX2 + dY2);
				addSequential(new PIDMove(distance, dt, sd, pidMoveSrc));
				double x = AutoUtils.position.getX();
				double y = AutoUtils.position.getY();
				AutoUtils.position.setX(point[0]);
				AutoUtils.position.setY(point[1]);
				AutoUtils.position.setRot(Math.toDegrees(Math.atan((point[0] - x) / (point[1] - y))));
			} else {
				throw new IllegalArgumentException();
			}
		}
	}

	public AutoMoveTo(String[] args) {
		this(args, Robot.dt, Robot.sd, Robot.dt.getDistEncAvg());
	}
}
