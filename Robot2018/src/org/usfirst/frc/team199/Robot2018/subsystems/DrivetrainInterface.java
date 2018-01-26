package org.usfirst.frc.team199.Robot2018.subsystems;

public interface DrivetrainInterface {
	/**
	 * Updates the PIDControllers' PIDConstants based on SmartDashboard values
	 */
	public void updatePidConstants();

	/**
	 * Activates the solenoid to push the drivetrain into low or high gear
	 * 
	 * @param forw
	 *            If the solenoid is to be pushed forward or not (backwards)
	 */
	public void changeShiftGear(boolean forw);

	/**
	 * Stops the solenoid that pushes the drivetrain into low or high gear
	 */
	public void turnGearSolenoidOff();

	/**
	 * Resets the AHRS value
	 */
	public void resetAHRS();

	/**
	 * Runs the left side of the drivetrain at the specified speed
	 * 
	 * @param value
	 *            Value for the motor(s) to run at
	 */
	public void setLeftMotor(double value);

	/**
	 * Runs the right side of the drivetrain at the specified speed
	 * 
	 * @param value
	 *            Value for the motor(s) to run at
	 */
	public void setRightMotor(double value);

	/**
	 * Tells the drivetrain to stop running
	 */
	public void stopDrive();

	/**
	 * Resets the encoders' distances to zero
	 */
	public void resetEnc();

	/**
	 * Drives based on joystick input and SmartDashboard values
	 */
	public void teleopDrive();

	/**
	 * Drives the robot based on parameters and SmartDashboard values
	 * 
	 * @param speed
	 *            The amount to move forward
	 * @param turn
	 *            The amount to turn
	 */
	public void arcadeDrive(double speed, double turn);

	/**
	 * Drive the robot based on parameters and SmartDashboard values
	 * 
	 * @param leftSpeed
	 *            The value to run the left of the drivetrain at
	 * @param rightSpeed
	 *            The value to run the right of the drivetrain at
	 */
	public void tankDrive(double leftSpeed, double rightSpeed);

	/**
	 * Used for getting the speed at which the left side of the drivetrain is
	 * currently running
	 * 
	 * @return The speed that the left side of the drivetrain is set to
	 */
	public double getDtLeft();

	/**
	 * Used for getting the speed at which the right side of the drivetrain is
	 * currently running
	 * 
	 * @return The speed that the right side of the drivetrain is set to
	 */
	public double getDtRight();

	/**
	 * Used to get the angle that the gyro currently reads
	 * 
	 * @return The angle that the gyro reads
	 */
	public double getGyroAngle();

	/**
	 * Resets the gyro to 0
	 */
	public void resetGyro();

	/**
	 * Disables the turnPID PIDController used for turning
	 */
	public void disableTurnPid();

	/**
	 * Enables the turnPID PIDController used for turning
	 */
	public void enableTurnPid();

	/**
	 * Sets the setPoint of the turnPID PIDController
	 * 
	 * @param set
	 *            The value to set the setPoint at
	 */
	public void setTurnSetpoint(double set);

	/**
	 * Enable the movePID PIDController used for moving
	 */
	public void enableMovePid();

	/**
	 * Disables the movePID PIDController used for moving
	 */
	public void disableMovePid();

	/**
	 * Sets the setPoint of the moveLeftPID PIDController
	 * 
	 * @param set
	 *            The value to set the setPoint at
	 */
	public void setMoveSetpointLeft(double set);

	/**
	 * Sets the setPoint of the moveRightPID PIDController
	 * 
	 * @param set
	 *            The value to set the setPoint at
	 */
	public void setMoveSetpointRight(double set);

	/**
	 * Sets the distancePerPulse property on the left encoder
	 * 
	 * @param dist
	 *            The distance to set the distancePerPulse at
	 */
	public void setDistancePerPulseLeft(double dist);

	/**
	 * Sets the distancePerPulse property on the right encoder
	 * 
	 * @param dist
	 *            The distance to set the distancePerPulse at
	 */
	public void setDistancePerPulseRight(double dist);

	/**
	 * Returns the value that Drivetrain receives due to implementing PIDOutput
	 * 
	 * @return The value that is written by PIDControllers
	 */
	public double getAnglePidOut();

	/**
	 * Returns the value that leftdrive should be set to according to PIDControllers
	 * 
	 * @return The value that is written by PIDControllers
	 */
	public double getLeftPidOut();

	/**
	 * Returns the value that rightdrive should be set to according to
	 * PIDControllers
	 * 
	 * @return The value that is written by PIDControllers
	 */
	public double getRightPidOut();

	/**
	 * Returns whether the turnController PIDController senses that it's on target
	 * 
	 * @return Whether the turnController PIDController is on target
	 */
	public boolean onTurnTarg();

	/**
	 * Returns whether the moveController PIDController senses that it's on target
	 * 
	 * @return Whether the moveController PIDController is on target
	 */
	public boolean onDriveTarg();
}
