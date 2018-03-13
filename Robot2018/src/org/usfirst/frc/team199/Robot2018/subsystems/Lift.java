package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.UpdateLiftPosition;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class Lift extends PIDSubsystem implements LiftInterface {

	private final SpeedControllerGroup liftMotors = RobotMap.liftMotors;
	private final Encoder liftEnc = RobotMap.liftEnc;
	private Position currPosition = Position.GROUND;

	private final int NUM_STAGES;
	private final double WIGGLE_ROOM;

	public Lift(String name, double kP, double kI, double kD, double kF) {
		super(name, kP, kI, kD, kF);
		NUM_STAGES = (int) Robot.getConst("Lift stages", 1);
		WIGGLE_ROOM = (int) Robot.getConst("Lift wiggle room", 3.0);
		enable();
	}

	/**
	 * Set the default command for a subsystem here.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new UpdateLiftPosition(this));
	}

	/**
	 * Sets the current position in the lift subsystem
	 * 
	 * @param newPosition
	 *            - the new position meant to be set
	 */
	@Override
	public void setCurrPosition(Position newPosition) {
		currPosition = newPosition;
	}

	/**
	 * Uses (insert sensor here) to detect the current lift position
	 */
	@Override
	public double getHeight() {
		return liftEnc.getDistance() * NUM_STAGES;
	}

	/**
	 * stops the lift
	 */
	@Override
	public void stopLift() {
		liftMotors.stopMotor();
	}

	/**
	 * gets current motor values
	 */
	@Override
	public double getLiftSpeed() {
		return liftMotors.get();
	}

	/**
	 * Runs lift motors at specified speed
	 * 
	 * @param speed
	 *            - desired speed to run at
	 */
	@Override
	public void runMotor(double output) {
		liftMotors.set(output);
	}

	/**
	 * Returns the position the lift is currently at
	 * 
	 * @return pos - current position
	 */
	@Override
	public Position getCurrPos() {
		return currPosition;
	}

	/**
	 * Resets the encoder
	 */
	@Override
	public void resetEnc() {
		liftEnc.reset();
	}

	/**
	 * Gets the number of stages variable
	 */
	@Override
	public int getNumStages() {
		return NUM_STAGES;
	}

	/**
	 * Gets the extra distance above the switch or scale we want to lift in inches
	 */
	@Override
	public double getWiggleRoom() {
		return WIGGLE_ROOM;
	}

	/**
	 * Uses AMT103 Encoder to detect the current lift height with respect to the
	 * lift's min height (inches)
	 */
	@Override
	protected double returnPIDInput() {
		// return getHeight(); //Use this instead? What's the difference?
		return liftEnc.getDistance();
	}

	/**
	 * Runs the lift motors at the value that the pid loop calculated. Used
	 * internally by PIDSubsystem.
	 */
	@Override
	protected void usePIDOutput(double output) {
		double out = output;
		double spd = liftEnc.getRate();
		out += convertSpdToVoltage(spd);
		runMotor(out);
	}

	/**
	 * Takes a value for the current lift speed and translates it to the amount of
	 * voltage to motors need to supply.
	 * 
	 * @param speed
	 *            - the current lift speed (in/s)
	 */
	public double convertSpdToVoltage(double speed) {
		return 0;
	}
}
