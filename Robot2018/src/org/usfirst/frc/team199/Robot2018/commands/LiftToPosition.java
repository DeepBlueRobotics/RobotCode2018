package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.LiftHeight;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftToPosition extends Command {

	private Lift lift;
	private LiftHeight pos;
	private boolean goToGround;

	public LiftToPosition(Lift lift, LiftHeight goal) {
		requires(Robot.lift);
		this.lift = lift;
		pos = goal;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		lift.getPIDController().setPID(Robot.getConst("LiftkP", 0.1), Robot.getConst("LiftkI", 0),
				Robot.getConst("LiftkD", 0), Robot.getConst("LiftkF", 0.1));
		double setpoint = lift.getDesiredDistFromPos(pos);
		lift.setSetpoint(setpoint);
		System.out.println("Target Height: " + setpoint);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println("Current Height: " + lift.getHeight());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return lift.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
