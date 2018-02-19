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
	 * Sets the left roller to run at the specified speed
	 * 
	 * @param speed
	 *            Speed the left motor should run at
	 */
	public void runLeftIntake(double speed);

	/**
	 * Sets the right roller to run at the specified speed
	 * 
	 * @param speed
	 *            Speed the right motor should run at
	 */
	public void runRightIntake(double speed);

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
	 * Toggles the left intake between open and closed
	 */
	public void toggleLeftIntake();

	/**
	 * Toggles the right intake between open and closed
	 */
	public void toggleRightIntake();

	/**
	 * Closes the intake
	 */
	public void closeIntake();

	/**
	 * Opens the intake
	 */
	public void openIntake();
}
