package org.usfirst.frc.team199.Robot2018.subsystems;

import edu.wpi.first.wpilibj.PIDSource;

public interface DrivetrainInterface {

	public void initDefaultCommand();

	/**
	 * Returns the getRate() of the left encoder
	 * 
	 * @return the rate of the left encoder
	 */
	public double getLeftEncRate();

	/**
	 * Returns the getRate() of the right encoder
	 * 
	 * @return the rate of the right encoder
	 */
	public double getRightEncRate();

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
	 * currently set to. Gets data straight from SpeedControllerGroup.
	 * 
	 * @return The speed that the left side of the drivetrain is set to [-1, 1]
	 */
	public double getDtLeftSpeed();

	/**
	 * Used for getting the speed at which the right side of the drivetrain is
	 * currently set to. Gets data straight from SpeedControllerGroup.
	 * 
	 * @return The speed that the right side of the drivetrain is set to [-1, 1]
	 */
	public double getDtRightSpeed();

	/**
	 * Updates the PIDControllers' PIDConstants based on SmartDashboard values
	 */
	public void updatePidConstants();

	/**
	 * Enable the VelocityPIDControllers used for velocity control on each side of
	 * the DT
	 */
	public void enableVelocityPIDs();

	/**
	 * Disables the VelocityPIDControllers used for velocity control on each side of
	 * the DT
	 */
	public void disableVelocityPIDs();

	/**
	 * Resets the AHRS value
	 */
	public void resetAHRS();

	public double getGyroRate();

	/**
	 * Used to get the yaw angle (Z-axis in degrees) that the ahrs currently reads
	 * 
	 * @return The angle that the ahrs reads (in degrees)
	 */
	public double getAHRSAngle();

	/**
	 * Resets the encoders' distances to zero
	 */
	public void resetDistEncs();

	/**
	 * Sets the distancePerPulse property on the left encoder
	 * 
	 * @param ratio
	 *            The ratio to set the distancePerPulse to (real dist units/encoder
	 *            pulses)
	 */
	public void setDistancePerPulseLeft(double ratio);

	/**
	 * Sets the distancePerPulse property on the right encoder
	 * 
	 * @param ratio
	 *            The ratio to set the distancePerPulse to (real dist units/encoder
	 *            pulses)
	 */
	public void setDistancePerPulseRight(double ratio);

	/**
	 * Returns the distance (in real units) that the left encoder reads
	 * 
	 * @return How far the left encoder has traveled in real units since last reset
	 */
	public double getLeftEncDist();

	/**
	 * Returns the distance (in real units) that the right encoder reads
	 * 
	 * @return How far the right encoder has traveled in real units since last reset
	 */
	public double getRightEncDist();

	/**
	 * Activates the solenoid to push the drivetrain into high or low gear
	 * 
	 * @param highGear
	 *            If the solenoid is to be pushed into high gear (true, kForward) or
	 *            low gear (false, kReverse)
	 */
	public void shiftGears(boolean highGear);

	/**
	 * Stops the solenoid that pushes the drivetrain into low or high gear
	 */
	public void shiftGearSolenoidOff();

	/**
	 * Returns the gyroscope
	 * 
	 * @return the gyroscope
	 */
	public PIDSource getGyro();

	/**
	 * Reset the kf constants for both VelocityPIDControllers based on current DT
	 * gearing (high or low gear).
	 * 
	 * @param newKF
	 *            the new kF constant based on high and low gear max speeds; should
	 *            be 1 / max speed
	 * @return the new kF value as 1 / correct max speed
	 */
	public double resetVelocityPIDkFConsts();

	/**
	 * Gets the current max speed of the DT based on gearing (high or low gear)
	 * 
	 * @return the current max speed of the DT in inches/second
	 */
	public double getCurrentMaxSpeed();

	/**
	 * Put left and right velocity controllers (PID) on SmartDashboard.
	 */
	public void putVelocityControllersToDashboard();
}
