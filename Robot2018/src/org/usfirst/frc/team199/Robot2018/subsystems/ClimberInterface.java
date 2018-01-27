package org.usfirst.frc.team199.Robot2018.subsystems;

public interface ClimberInterface {
	
	/**
	 * Set the default command for a subsystem here.
	 * */
	public void initDefaultCommand();
	
	/**
	 * 
	 */
	public void runClimber();
	
	/**
	 * attaches the climber hook to the lift.
	 * Requires that Lift is on the ground
	 */
	public void attachToLift();
	
	/**
	 * winches upwards
	 */
	public void goUp();
	
	/**
	 * attaches hook to bar and releases it from the lift
	 */
	public void attachToBar();
	
	/**
	 * stops the climber
	 */
	public void stopClimber();

	
}
