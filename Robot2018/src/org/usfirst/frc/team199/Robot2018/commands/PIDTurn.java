package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PIDTurn extends Command implements PIDOutput {

	double target;
	DrivetrainInterface dt;
	private PIDController turnController;

	public PIDTurn(double targ, DrivetrainInterface dt, PIDSource ahrs, SmartDashboardInterface sd) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		target = targ;
		this.dt = dt;
		if (Robot.dt != null) {
			requires(Robot.dt);
		}
		turnController = new PIDController(sd.getConst("TurnkP", 1), sd.getConst("TurnkI", 0),
			sd.getConst("TurnkD", 0), ahrs, this);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		dt.resetAHRS();
		turnController.disable();
		turnController.setInputRange(-180, 180);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setContinuous();
		turnController.setAbsoluteTolerance(Robot.getConst("TurnTolerance", 1));
		turnController.setSetpoint(target);
		turnController.enable();
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
		return turnController.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		turnController.disable();
		turnController.free();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	@Override
	public void pidWrite(double output) {
		dt.arcadeDrive(0, output);
	}
}
