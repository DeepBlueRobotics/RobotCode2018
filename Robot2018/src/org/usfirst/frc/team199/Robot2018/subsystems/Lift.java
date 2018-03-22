package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.UpdateLiftPosition;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
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
	private final double GROUND_DIST;
	private final double HOLD_CUBE_DIST;
	private final double SWITCH_DIST;
	private final double SCALE_DIST;
	private final double BAR_DIST;

	private LiftHeight desiredPos;
	private boolean goToGround;

	public Lift() {
		super("Lift", 0, 0, 0, 0);

		double maxSpeed = getLiftMaxSpeed();

		System.out.println("Lift Time Const: " + getLiftTimeConstant());

		double r = Robot.getConst("LiftPidR", 0.3);
		double kP = r / getLiftTimeConstant() / maxSpeed;
		double kI = 0;
		double kD = r / maxSpeed;
		double kF = 1 / maxSpeed * Robot.getConst("Default PID Update Time", 0.05);

		PIDController liftController = getPIDController();

		// liftController.setPID(Robot.getConst("LiftkP", kP), Robot.getConst("LiftkI",
		// kI), Robot.getConst("LiftkD", kD),
		// Robot.getConst("LiftkF", kF));
		liftController.setPID(kP, kI, kD, kF);

		setInputRange(0, Robot.getConst("Lift Max Height", 24));
		setOutputRange(-1, 1);
		setAbsoluteTolerance(Robot.getConst("Lift Tolerance", 0.8));

		// dpp = (pitch circumference of sprocket) / (pulses per rev of output shaft)
		double dpp = 2 * Math.PI * getLiftRadius() / 2048;
		liftEnc.setDistancePerPulse(dpp);

		NUM_STAGES = (int) Robot.getConst("Lift stages", 1);
		WIGGLE_ROOM = (int) Robot.getConst("Lift wiggle room", 3.0); // inches

		// calculate constant measurements
		GROUND_DIST = 0.7;
		HOLD_CUBE_DIST = 4;
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

	/**
	 * @return whether or not the cube
	 */
	private boolean atHoldCube() {
		double tol = Robot.getConst("Lift Tolerance", 0.8);
		double height = getHeight();
		// true if (height within tolerance of HOLD_CUBE height) and (lift is not
		// moving)
		return height <= (HOLD_CUBE_DIST + tol) && height >= (HOLD_CUBE_DIST - tol) && getSpeed() <= 0.1;
	}

	/**
	 * @return true if reached target height and does not still have to go to the
	 *         ground as a secondary, consecutive target
	 */
	// @Override
	// public boolean onTarget() {
	// double targ = getSetpoint();
	// double tol = Robot.getConst("Lift Tolerance", 0.8);
	// double height = getHeight();
	// // true if (height within tolerance of targ) and (lift is not moving)
	// boolean wInTol = height <= (targ + tol) && height >= (targ - tol) &&
	// getSpeed() <= 0.1;
	// return wInTol && !goToGround;
	// }

	/**
	 * Set the lift's setpoint to this value continuously.
	 * 
	 * @param pos
	 *            - the desired FINAL height
	 * @return the FIRST height to travel to; in all cases but GROUND, FIRST = FINAL
	 *         targ
	 */
	public double getDesiredDistFromPos(LiftHeight pos) {
		double desiredDist;
		LiftHeight height = pos;
		switch (height) {
		case GROUND:
			// if (atHoldCube() || goToGround) {
			desiredDist = GROUND_DIST;
			// false bc doesn't need to hold the GROUND as a second consecutive targ anymore
			goToGround = true;
			break;
		// } else {
		// // falls through to HOLD_CUBE case
		// height = LiftHeight.HOLD_CUBE;
		// goToGround = false;
		// }
		case HOLD_CUBE:
			desiredDist = HOLD_CUBE_DIST;
			// goToGround = false;
			break;
		case SWITCH:
			desiredDist = SWITCH_DIST;
			// goToGround = false;
			break;
		case SCALE:
			desiredDist = SCALE_DIST;
			goToGround = false;
			break;
		case BAR:
			desiredDist = BAR_DIST;
			// goToGround = false;
			break;
		default:
			desiredDist = GROUND_DIST;
			// goToGround = false;
			break;
		}

		return desiredDist;
	}

	/**
	 * @return the desired LiftHeight to be at
	 */
	public LiftHeight desiredPos() {
		return desiredPos;
	}

	/**
	 * @return the absolute rate of the lift encoder (in/s)
	 */
	public double getSpeed() {
		return Math.abs(liftEnc.getRate());
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
	 * Sends specific voltage to lift motor, clamped at max voltage
	 * 
	 * @param output
	 *            - desired voltage to set motor to [-1, 1]
	 */
	@Override
	public void runMotor(double output) {
		double absMax = Robot.getConst("Lift Max Voltage", 0.5);
		double out = output;
		if (Math.abs(out) > absMax) {
			out = Math.signum(out) * absMax;
		}
		liftMotors.set(out);
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
	 * @return the max speed of the lift (in/s)
	 */
	public double getLiftMaxSpeed() {
		double maxSpd = RobotMap.getOmegaMax() / getLiftGearRatio() / 60 * 2 * Math.PI * getLiftRadius();
		return Robot.getConst("Lift Max Speed", maxSpd);
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
		out += Robot.getConst("Lift: Necessary Voltage", 0.25);
		runMotor(out);
	}

	/**
	 * Gets the time constant of the drivetrain, which is used to calculate PID
	 * constants.
	 * 
	 * @return time constant
	 */
	public double getLiftTimeConstant() {
		double gearReduction = getLiftGearRatio();
		double radius = convertInToM(getLiftRadius());
		double timeConstant = RobotMap.getOmegaMax() / gearReduction / 60 * 2 * Math.PI
				* RobotMap.convertNtokG(getLiftedWeight()) * radius * radius
				/ (RobotMap.getStallTorque() * gearReduction);
		return timeConstant;
	}

	/**
	 * @param inches
	 *            - an amount in inches
	 * @return the equivalent amount in meters
	 */
	public double convertInToM(double inches) {
		return inches * 2.54 / 100;
	}

	/**
	 * @return the gear ratio of the lift gearbox
	 */
	public double getLiftGearRatio() {
		return Robot.getConst("Lift Gear Reduction", 10);
	}

	/**
	 * @return the lift sprocket's radius (inches)
	 */
	public double getLiftRadius() {
		// 15 tooth, #35 sprocket
		return Robot.getConst("Lift Sprocket Pitch Diam", 1.79) / 2;
	}

	/**
	 * @return the weight (in Newtons) of everything that is being lifted: intake,
	 *         lift components, NOT cube
	 */
	public double getLiftedWeight() {
		return Robot.getConst("Weight of Lifted Stuff", 62.91);
	}
}
