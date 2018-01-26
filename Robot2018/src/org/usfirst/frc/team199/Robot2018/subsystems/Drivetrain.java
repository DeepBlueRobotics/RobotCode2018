/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.TeleopDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends Subsystem implements PIDOutput, DrivetrainInterface {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final Encoder leftEnc = RobotMap.leftEnc;
	private final Encoder rightEnc = RobotMap.rightEnc;
	private final SpeedControllerGroup dtLeft = RobotMap.dtLeft;
	private final SpeedControllerGroup dtRight = RobotMap.dtRight;
	private final DifferentialDrive robotDrive = RobotMap.robotDrive;
	private final PIDController turnController = RobotMap.turnController;
	// private final PIDController moveController = RobotMap.moveController;
	private final PIDController moveLeftController = RobotMap.moveLeftController;
	private final PIDController moveRightController = RobotMap.moveRightController;

	private final AHRS ahrs = RobotMap.ahrs;
	private final AnalogGyro dtGyro = RobotMap.dtGyro;
	private final DoubleSolenoid dtGear = RobotMap.dtGear;

	private double anglePidOut = 0;
	private double leftPidOut = 0;
	private double rightPidOut = 0;

	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}

	/**
	 * Updates the PIDControllers' PIDConstants based on SmartDashboard values
	 */
	public void updatePidConstants() {
		turnController.setPID(Robot.getConst("TurnkP", 1), Robot.getConst("TurnkI", 0), Robot.getConst("TurnkD", 0));
		moveLeftController.setPID(Robot.getConst("MoveLeftkP", 1), Robot.getConst("MoveLeftkI", 0),
				Robot.getConst("MoveLeftkD", 0));
		moveRightController.setPID(Robot.getConst("MoveRightkP", 1), Robot.getConst("MoveRightkI", 0),
				Robot.getConst("MoveRightkD", 0));
	}

	/**
	 * Activates the solenoid to push the drivetrain into low or high gear
	 * 
	 * @param forw
	 *            If the solenoid is to be pushed forward or not (backwards)
	 */
	public void changeShiftGear(boolean forw) {
		if (forw ^ Robot.getBool("Drivetrain Gear Shift Backwards", false)) {
			dtGear.set(DoubleSolenoid.Value.kForward);
		} else {
			dtGear.set(DoubleSolenoid.Value.kReverse);
		}
	}

	/**
	 * Stops the solenoid that pushes the drivetrain into low or high gear
	 */
	public void turnGearSolenoidOff() {
		dtGear.set(DoubleSolenoid.Value.kOff);
	}

	/**
	 * Resets the AHRS value
	 */
	public void resetAHRS() {
		ahrs.reset();
	}

	/**
	 * Runs the left side of the drivetrain at the specified speed
	 * 
	 * @param value
	 *            Value for the motor(s) to run at
	 */
	public void setLeftMotor(double value) {
		dtLeft.set(value);
	}

	/**
	 * Tells the left side of the drivetrain to stop running
	 */
	private void stopLeftMotor() {
		dtLeft.stopMotor();
	}

	/**
	 * Runs the right side of the drivetrain at the specified speed
	 * 
	 * @param value
	 *            Value for the motor(s) to run at
	 */
	public void setRightMotor(double value) {
		dtRight.set(value);
	}

	/**
	 * Tells the right side of the drivetrain to stop running
	 */
	private void stopRightMotor() {
		dtRight.stopMotor();
	}

	/**
	 * Tells the drivetrain to stop running
	 */
	public void stopDrive() {
		stopRightMotor();
		stopLeftMotor();
	}

	/**
	 * Resets the encoders' distances to zero
	 */
	public void resetEnc() {
		leftEnc.reset();
		rightEnc.reset();
	}

	/**
	 * Drives based on joystick input and SmartDashboard values
	 */
	public void teleopDrive() {
		if (Robot.getBool("Arcade Drive", true)) {
			if (Robot.getBool("Arcade Drive Default Setup", true)) {
				Robot.dt.arcadeDrive(Robot.oi.leftJoy.getY(), Robot.oi.rightJoy.getX());
			} else {
				Robot.dt.arcadeDrive(Robot.oi.rightJoy.getY(), Robot.oi.leftJoy.getX());
			}
		} else {
			Robot.dt.tankDrive(Robot.oi.leftJoy.getY(), Robot.oi.rightJoy.getY());
		}
	}

	/**
	 * Drives the robot based on parameters and SmartDashboard values
	 * 
	 * @param speed
	 *            The amount to move forward
	 * @param turn
	 *            The amount to turn
	 */
	public void arcadeDrive(double speed, double turn) {
		robotDrive.arcadeDrive(speed, turn, Robot.getBool("Square Drive Values", false));
	}

	/**
	 * Drive the robot based on parameters and SmartDashboard values
	 * 
	 * @param leftSpeed
	 *            The value to run the left of the drivetrain at
	 * @param rightSpeed
	 *            The value to run the right of the drivetrain at
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed, Robot.getBool("Square Drive Values", false));
	}

	/**
	 * Used for getting the speed at which the left side of the drivetrain is
	 * currently running
	 * 
	 * @return The speed that the left side of the drivetrain is set to
	 */
	public double getDtLeft() {
		return dtLeft.get();
	}

	/**
	 * Used for getting the speed at which the right side of the drivetrain is
	 * currently running
	 * 
	 * @return The speed that the right side of the drivetrain is set to
	 */
	public double getDtRight() {
		return dtRight.get();
	}

	/**
	 * Used to get the angle that the gyro currently reads
	 * 
	 * @return The angle that the gyro reads
	 */
	public double getGyroAngle() {
		return dtGyro.getAngle();
	}

	/**
	 * Resets the gyro to 0
	 */
	public void resetGyro() {
		dtGyro.reset();
	}

	/**
	 * Disables the turnPID PIDController used for turning
	 */
	public void disableTurnPid() {
		turnController.disable();
	}

	/**
	 * Enables the turnPID PIDController used for turning
	 */
	public void enableTurnPid() {
		turnController.enable();
	}

	/**
	 * Sets the setPoint of the turnPID PIDController
	 * 
	 * @param set
	 *            The value to set the setPoint at
	 */
	public void setTurnSetpoint(double set) {
		turnController.setSetpoint(set);
	}

	/**
	 * Enable the movePID PIDController used for moving
	 */
	public void enableMovePid() {
		moveLeftController.enable();
		moveRightController.enable();
	}

	/**
	 * Disables the movePID PIDController used for moving
	 */
	public void disableMovePid() {
		moveLeftController.disable();
		moveRightController.enable();
	}

	/**
	 * Sets the setPoint of the moveLeftPID PIDController
	 * 
	 * @param set
	 *            The value to set the setPoint at
	 */
	public void setMoveSetpointLeft(double set) {
		moveLeftController.setSetpoint(set);
	}

	/**
	 * Sets the setPoint of the moveRightPID PIDController
	 * 
	 * @param set
	 *            The value to set the setPoint at
	 */
	public void setMoveSetpointRight(double set) {
		moveRightController.setSetpoint(set);
	}

	/**
	 * Sets the distancePerPulse property on the left encoder
	 * 
	 * @param dist
	 *            The distance to set the distancePerPulse at
	 */
	public void setDistancePerPulseLeft(double dist) {
		leftEnc.setDistancePerPulse(dist);
	}

	/**
	 * Sets the distancePerPulse property on the right encoder
	 * 
	 * @param dist
	 *            The distance to set the distancePerPulse at
	 */
	public void setDistancePerPulseRight(double dist) {
		rightEnc.setDistancePerPulse(dist);
	}

	/**
	 * Returns the value that Drivetrain receives due to implementing PIDOutput
	 * 
	 * @return The value that is written by PIDControllers
	 */
	public double getAnglePidOut() {
		return anglePidOut;
	}
	
	/**
	 * Returns the value that leftdrive should be set to according to PIDControllers
	 * 
	 * @return The value that is written by PIDControllers
	 */
	public double getLeftPidOut() {
		return leftPidOut;
	}
	
	/**
	 * Returns the value that rightdrive should be set to according to PIDControllers
	 * 
	 * @return The value that is written by PIDControllers
	 */
	public double getRightPidOut() {
		return rightPidOut;
	}

	/**
	 * Returns the distance that the left encoder reads
	 * 
	 * @return How much the left encoder has travelled
	 */
	public double getLeftEnc() {
		return leftEnc.getDistance();
	}

	/**
	 * Returns the distance that the right encoder reads
	 * 
	 * @return How much the right encoder has travelled
	 */
	public double getRightEnc() {
		return rightEnc.getDistance();
	}
	// public boolean onTargetLeft() {
	// return moveLeftController.onTarget();
	// }
	// public boolean onTargetRight() {
	// return moveRightController.onTarget();
	// }

	@Override
	public void pidWrite(double output) {
		anglePidOut = output;
	}

	/**
	 * Sets the leftPidOutput
	 * 
	 * @param output
	 *            The value to set the output to
	 */
	public void pidLeftWrite(double output) {
		leftPidOut = output;
	}

	/**
	 * Sets the rightPidOutput
	 * 
	 * @param output
	 *            The value to set the output to
	 */
	public void pidRightWrite(double output) {
		rightPidOut = output;
	}

	/**
	 * Returns the average of two numbers
	 * 
	 * @param a
	 *            First number to average
	 * @param b
	 *            Second number to average
	 * @return The arithmetic mean of a and b
	 */
	public static double average(double a, double b) {
		return (a + b) / 2;
	}

	/**
	 * Returns whether the turnController PIDController senses that it's on target
	 * 
	 * @return Whether the turnController PIDController is on target
	 */
	public boolean onTurnTarg() {
		return turnController.onTarget();
	}

	/**
	 * Returns whether the moveLeftController and moveRightController PIDController
	 * sense that their on target
	 * 
	 * @return Whether the moveController PIDControllers are on target
	 */
	public boolean onDriveTarg() {
		return moveLeftController.onTarget() && moveRightController.onTarget();
	}

}
