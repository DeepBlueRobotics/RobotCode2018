package org.usfirst.frc.team199.Robot2018.subsystems;

public interface IntakeEjectInterface {

	/**
	 * Set the default command for a subsystem here.
	 */
	public void initDefaultCommand();

	/**
	 * returns current left motor value
	 */
	public double getLeftIntakeSpeed();

	/**
	 * returns current right motor value
	 */
	public double getRightIntakeSpeed();

	/**
	 * Uses current to check if the wheels are blocked aka the cube is inside the
	 * robot
	 * 
	 */
	public boolean hasCube();

	/**
	 * stops the motors
	 * 
	 */
	public void stopIntake();

	/**
	 * Spins the rollers
	 * 
	 * @param speed
	 *            - positive -> rollers in, negative -> rollers out
	 */
	public void runIntake(double speed);

	/**
	 * Raises the intake
	 */
	public void raiseIntake();

	/**
	 * Lowers the intake
	 */
	public void lowerIntake();

	/**
	 * Closes the intake
	 */
	public void closeIntake();

	/**
	 * Opens the intake
	 */
	public void openIntake();
}
