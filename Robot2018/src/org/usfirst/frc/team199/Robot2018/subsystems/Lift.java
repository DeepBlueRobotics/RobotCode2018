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
	private LiftHeight currPosition = LiftHeight.GROUND;

	private final int NUM_STAGES;
	private final double WIGGLE_ROOM;
	private final double SWITCH_DIST;
	private final double SCALE_DIST;
	private final double BAR_DIST;

	public Lift() {
		super("Lift", Robot.getConst("LiftkP", 0.1), Robot.getConst("LiftkI", 0), Robot.getConst("LiftkD", 0),
				Robot.getConst("LiftkF", 0.1));

		setInputRange(0, Robot.getConst("Lift Max Height", 24));
		setOutputRange(-1, 1);

		NUM_STAGES = (int) Robot.getConst("Lift stages", 1);
		WIGGLE_ROOM = (int) Robot.getConst("Lift wiggle room", 3.0); // inches

		// calculate constant measurements
		// distance to switch 18.75 inches in starting position
		SWITCH_DIST = (18.75 + WIGGLE_ROOM) / NUM_STAGES;
		// distance to scale 5 feet starting 63 / 3 = 21
		SCALE_DIST = (60.0 + WIGGLE_ROOM) / NUM_STAGES;
		// 7 feet starting; bar distance should be changed because I'm not aware how
		// climber mech will be positioned
		BAR_DIST = (84.0 + WIGGLE_ROOM) / NUM_STAGES;
		enable();
	}

	/**
	 * Set the default command for a subsystem here.
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new UpdateLiftPosition(this));
	}

	public double getDesiredDistFromPos(LiftHeight pos) {
		double desiredDist;
		switch (pos) {
		case GROUND:
			desiredDist = 0;
			break;
		case SWITCH:
			desiredDist = SWITCH_DIST;
			break;
		case SCALE:
			desiredDist = SCALE_DIST;
			break;
		case BAR:
			desiredDist = BAR_DIST;
			break;
		default:
			desiredDist = 0;
			break;
		}

		return desiredDist;
	}

	/**
	 * Sets the current position in the lift subsystem
	 * 
	 * @param newPosition
	 *            - the new position meant to be set
	 */
	@Override
	public void setCurrPosition(LiftHeight newPosition) {
		currPosition = newPosition;
	}

	/**
	 * Uses AMT103 Encoder to detect the current lift position
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
	 *            - desired speed to run at [-1, 1]
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
	public LiftHeight getCurrPos() {
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
		return getHeight();
	}

	/**
	 * Runs the lift motors at the value that the pid loop calculated. Used
	 * internally by PIDSubsystem.
	 */
	@Override
	protected void usePIDOutput(double output) {
		double out = output;
		double spd = liftEnc.getRate();
		out += Robot.getConst("Lift: Necessary Voltage", 0);
		runMotor(out);
	}
}
