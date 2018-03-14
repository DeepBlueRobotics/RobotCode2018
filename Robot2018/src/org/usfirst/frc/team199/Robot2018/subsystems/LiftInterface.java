package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.Position;

public interface LiftInterface {
	
	/**
	 * Set the default command for a subsystem here.
	 * */
	public void initDefaultCommand();
	
	public enum Position {
		GROUND,
		SWITCH,
		SCALE,
		BAR
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
	 * @param speed - desired speed to run at
	 */
	public void runMotor(double speed);
	
	/**
	 * Returns the position the lift is currently at
	 * @return pos - current position
	 */
	public Position getCurrPos();
	
	/**
	 * Resets the encoder
	 */
	public void resetEnc();
	
	/**
	 * Sets the current position in the lift subsystem
	 * @param newPosition - the new position meant to be set
	 */
	public void setCurrPosition(Position newPosition);
	
	
}
