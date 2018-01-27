package org.usfirst.frc.team199.Robot2018.subsystems;

public interface IntakeEjectInterface {
	
	/**
	 * Set the default command for a subsystem here.
	 * */
	public void initDefaultCommand();
	
	/**
	 * returns current motor value
	 */
	public double getIntake();
	
	/**
	 * Uses (insert sensor here) to detect 
	 * a cube in front of the robot.
	 */
	public boolean detectCube();
	
	/**
	 * Uses (insert sensor here) to detect if 
	 * the cube is currently inside the robot
	 * 
	 */
	public boolean hasCube();
	
	/**
	 * stops the motors
	 * 
	 */
	public boolean stopIntake();
	
	/**
	 * Spins the rollers
	 * @param speed - positive -> rollers in, negative -> rollers out
	 */
	public void runIntake(double speed);
	
	
}
