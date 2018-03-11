package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.AutoLift;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem implements LiftInterface {

	private final SpeedControllerGroup liftMotors = RobotMap.liftMotors;
	private final Encoder liftEnc = RobotMap.liftEnc;
	private Position currPosition = Position.GROUND;

	private final int NUM_STAGES;
	private final double WIGGLE_ROOM;

	public Lift() {
		NUM_STAGES = (int) Robot.getConst("Lift stages", 1);
		WIGGLE_ROOM = (int) Robot.getConst("Lift wiggle room", 3.0);
	}

	/**
	 * Set the default command for a subsystem here.
	 */
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		int angle = Robot.oi.manipulator.getPOV();

		Command com = null;

		if (angle == 0) {
			com = new AutoLift(Position.GROUND, this);
			setDefaultCommand(com);
		} else if (angle == 90 || angle == 180 || angle == 270) {
			com = new AutoLift(Position.SWITCH, this);
			setDefaultCommand(com);
		}
	}

	/**
	 * Sets the current position in the lift subsystem
	 * 
	 * @param newPosition
	 *            - the new position meant to be set
	 */
	public void setCurrPosition(Position newPosition) {
		currPosition = newPosition;
	}

	/**
	 * Uses (insert sensor here) to detect the current lift position
	 */
	public double getHeight() {
		return liftEnc.getDistance() * NUM_STAGES;
	}

	/**
	 * stops the lift
	 */
	public void stopLift() {
		liftMotors.stopMotor();
	}

	/**
	 * gets current motor values
	 */
	public double getLiftSpeed() {
		return liftMotors.get();
	}

	/**
	 * Runs lift motors at specified speed
	 * 
	 * @param speed
	 *            - desired speed to run at
	 */
	public void runMotor(double output) {
		liftMotors.set(output);
	}

	/**
	 * Returns the position the lift is currently at
	 * 
	 * @return pos - current position
	 */
	public Position getCurrPos() {
		return currPosition;
	}

	/**
	 * Resets the encoder
	 */
	public void resetEnc() {
		liftEnc.reset();
	}

	/**
	 * Gets the number of stages variable
	 */
	public int getNumStages() {
		return NUM_STAGES;
	}

	/**
	 * Gets the extra distance above the switch or scale we want to lift in inches
	 */
	public double getWiggleRoom() {
		return WIGGLE_ROOM;
	}
}
