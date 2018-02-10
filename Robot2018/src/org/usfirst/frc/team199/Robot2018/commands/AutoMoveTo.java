package org.usfirst.frc.team199.Robot2018.commands;

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
		String parentheseless;
		String[] pointparts;
		for (String arg : args) {
			if (AutoUtils.isDouble(arg)) {
				rotation = Double.valueOf(arg);
				addSequential(new PIDTurn(rotation - AutoUtils.getRot(), dt, sd, pidMoveSrc));
				AutoUtils.setRot(rotation);
			} else if (AutoUtils.isPoint(arg)) {
				parentheseless = arg.substring(1, arg.length() - 1);
				pointparts = parentheseless.split(",");
				point[0] = Double.parseDouble(pointparts[0]);
				point[1] = Double.parseDouble(pointparts[1]);
				addSequential(new PIDTurn(Math.toDegrees(
						Math.atan((point[0] - AutoUtils.getX()) / (point[1] - AutoUtils.getY())) - AutoUtils.getRot()),
						dt, sd, pidMoveSrc));
				addSequential(new PIDMove(
						Math.sqrt(((point[0] - AutoUtils.getX()) * (point[0] - AutoUtils.getX())
								+ ((point[1] - AutoUtils.getY()) * (point[1] - AutoUtils.getY())))),
						dt, sd, pidMoveSrc));
				double x = AutoUtils.getX();
				double y = AutoUtils.getY();
				AutoUtils.setX(point[0]);
				AutoUtils.setY(point[1]);
				AutoUtils.setRot(Math.toDegrees(Math.atan((point[0] - x) / (point[1] - y))));
			} else {
				throw new IllegalArgumentException();
			}
		}
	}
}
