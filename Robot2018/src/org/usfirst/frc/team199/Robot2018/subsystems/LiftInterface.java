package org.usfirst.frc.team199.Robot2018.subsystems;

public interface LiftInterface {
	
	/**
	 * Set the default command for a subsystem here.
	 * */
	public void initDefaultCommand();
	
	/**
	 * Uses (insert sensor here) to detect the distance above the ground
	 */
	public double getDistance();
	
	/**
	 * stops the lift
	 */
	public void stopLift();
	
	/**
	 * gets current motor values
	 */
	public double getLift();
	
	/**
	 * goes to the bottom
	 */
	public void goToGround();
	
	/**
	 * goes to switch height
	 */
	public void goToSwitch();
	
	/**
	 * goes to scale height
	 * @param offset - the distance up or down from standard scale height
	 */
	public void goToScale(double offset);
	
	/**
	 * goes to bar height
	 */
	public void goToBar();
	
}
