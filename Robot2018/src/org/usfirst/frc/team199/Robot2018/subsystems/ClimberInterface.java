package org.usfirst.frc.team199.Robot2018.subsystems;

public interface ClimberInterface {
	
	/**
	 * Set the default command for a subsystem here.
	 * */
	public void initDefaultCommand();
	
	/**
	 * runs the motors
	 */
	public void runClimber(double speed);
	
	/**
	 * attaches the climber hook to the lift.
	 * Requires that Lift is on the ground
	 */
	public void attachToLift();
	
	/**
	 * attaches hook to bar and releases it from the lift
	 */
	public void attachToBar();
	
	/**
	 * stops the climber
	 */
	public void stopClimber();

	
}
