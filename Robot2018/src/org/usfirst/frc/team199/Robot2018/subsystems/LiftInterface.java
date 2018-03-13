package org.usfirst.frc.team199.Robot2018.subsystems;

public interface LiftInterface {

	/**
	 * Set the default command for a subsystem here.
	 */
	public void initDefaultCommand();

	public enum LiftHeight {
		GROUND, SWITCH, SCALE, BAR;

		public static LiftHeight toLH(String str) {
			LiftHeight lh = null;
			switch (str) {
			case "GROUND":
				lh = GROUND;
				break;
			case "SWITCH":
				lh = SWITCH;
				break;
			case "SCALE":
				lh = SCALE;
				break;
			case "BAR":
				lh = BAR;
				break;
			default:
				lh = GROUND;
				break;
			}
			return lh;
		}
	}

	/**
	 * Uses (insert sensor here) to detect the current lift position
	 */
	public double getHeight();

	/**
	 * stops the lift
	 */
	public void stopLift();

	/**
	 * gets current motor values
	 */
	public double getLiftSpeed();

	/**
	 * Runs lift motors at specified speed
	 * 
	 * @param speed
	 *            - desired speed to run at
	 */
	public void runMotor(double speed);

	/**
	 * Returns the position the lift is currently at
	 * 
	 * @return pos - current position
	 */
	public LiftHeight getCurrPos();

	/**
	 * Resets the encoder
	 */
	public void resetEnc();

	/**
	 * Sets the current position in the lift subsystem
	 * 
	 * @param newPosition
	 *            - the new position meant to be set
	 */
	public void setCurrPosition(LiftHeight newPosition);

	/**
	 * Gets the number of stages variable
	 */
	public int getNumStages();

	/**
	 * Gets the extra distance above the switch or scale we want to lift in inches
	 */
	public double getWiggleRoom();
}
