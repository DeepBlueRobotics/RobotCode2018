package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDMove extends Command implements PIDOutput {

	private double target;
	private DrivetrainInterface dt;
	private PIDController moveController;

	public PIDMove(double targ, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource avg) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		target = targ;
		this.dt = dt;
		if (Robot.dt != null) {
			requires(Robot.dt);
		}
		moveController = new PIDController(sd.getConst("MovekP", 1), sd.getConst("MovekI", 0),
			sd.getConst("MovekD", 0), avg, this);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		dt.resetDistEncs();
		moveController.setInputRange(0, Double.MAX_VALUE);
		moveController.setOutputRange(-1.0, 1.0);
		moveController.setContinuous(false);
		moveController.setAbsoluteTolerance(Robot.getConst("MoveTolerance", 2));
		moveController.setSetpoint(target);
		moveController.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// This method is empty bc the moveController runs on a different thread as soon
		// as it is enabled.
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return moveController.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		moveController.disable();
		moveController.free();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	@Override
	public void pidWrite(double output) {
		dt.arcadeDrive(output, 0);
	}
}
