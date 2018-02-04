/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;
import org.usfirst.frc.team199.Robot2018.autonomous.VelocityPIDController;
import org.usfirst.frc.team199.Robot2018.commands.TeleopDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem implements DrivetrainInterface {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final Encoder leftEncDist = RobotMap.leftEncDist;
	private final Encoder rightEncDist = RobotMap.rightEncDist;
	private final Encoder leftEncRate = RobotMap.leftEncRate;
	private final Encoder rightEncRate = RobotMap.rightEncRate;
	private final PIDSourceAverage distEncAvg = RobotMap.distEncAvg;
	public final SpeedControllerGroup dtLeft = RobotMap.dtLeft;
	public final SpeedControllerGroup dtRight = RobotMap.dtRight;
	private final DifferentialDrive robotDrive = RobotMap.robotDrive;
	private final VelocityPIDController leftVelocityController = RobotMap.leftVelocityController;
	private final VelocityPIDController rightVelocityController = RobotMap.rightVelocityController;

	private final AHRS fancyGyro = RobotMap.fancyGyro;
	private final DoubleSolenoid dtGear = RobotMap.dtGear;

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}
	
	public void setLeft(double spd) {
		dtLeft.set(spd);
	}
	
	public void setRight(double spd) {
		dtRight.set(spd);
	}
	
	/**
	 * Use for testing only (i.e. when not going through robotDrive)
	 * */
	public void setVPIDs(double realSpd) {
		leftVelocityController.set(realSpd);
		rightVelocityController.set(-realSpd);
	}
	
	public double getLeftVPIDerror() {
		return leftVelocityController.getError();
	}

	public double getRightVPIDerror() {
		return rightVelocityController.getError();
	}
	
	public double getLeftVPIDSetpoint() {
		return leftVelocityController.get();
	}

	public double getRightVPIDSetpoint() {
		return rightVelocityController.get();
	}
	
	public double getLeftEncRate() {
		return leftEncRate.getRate();
	}
	
	public double getRightEncRate() {
		return rightEncRate.getRate();
	}

	/**
	 * Drives based on joystick input and SmartDashboard values
	 */
	@Override
	public void teleopDrive() {
		if (Robot.getBool("Arcade Drive", true)) {
			if (Robot.getBool("Arcade Drive Default Setup", true)) {
				Robot.dt.arcadeDrive(-Robot.oi.leftJoy.getY(), Robot.oi.rightJoy.getX());
			} else {
				Robot.dt.arcadeDrive(-Robot.oi.rightJoy.getY(), Robot.oi.leftJoy.getX());
			}
		} else {
			Robot.dt.tankDrive(-Robot.oi.leftJoy.getY(), -Robot.oi.rightJoy.getY());
		}
		SmartDashboard.putNumber("Drivetrain/Left VPID Targ", leftVelocityController.getSetpoint());
		SmartDashboard.putNumber("Drivetrain/Right VPID Targ", rightVelocityController.getSetpoint());
		SmartDashboard.putNumber("Drivetrain/Current Max Speed", getCurrentMaxSpeed());
		SmartDashboard.putNumber("Drivetrain/Left Enc Dist", leftEncDist.getDistance());
		SmartDashboard.putNumber("Drivetrain/Left Enc Rate", leftEncRate.getRate());
		SmartDashboard.putNumber("Drivetrain/Right Enc Dist", rightEncDist.getDistance());
		SmartDashboard.putNumber("Drivetrain/Right Enc Rate", rightEncRate.getRate());
		SmartDashboard.putNumber("Drivetrain/Enc Avg Dist", distEncAvg.pidGet());
	}

	/**
	 * Drives the robot based on parameters and SmartDashboard values
	 * 
	 * @param speed
	 *            The amount to move forward
	 * @param turn
	 *            The amount to turn
	 */
	@Override
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
	@Override
	public void tankDrive(double leftSpeed, double rightSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed, Robot.getBool("Square Drive Values", false));
	}

	/**
	 * Used for getting the speed at which the left side of the drivetrain is
	 * currently set to. Gets data straight from SpeedControllerGroup.
	 * 
	 * @return The speed that the left side of the drivetrain is set to [-1, 1]
	 */
	@Override
	public double getDtLeftSpeed() {
		return dtLeft.get();
	}

	/**
	 * Used for getting the speed at which the right side of the drivetrain is
	 * currently set to. Gets data straight from SpeedControllerGroup.
	 * 
	 * @return The speed that the right side of the drivetrain is set to [-1, 1]
	 */
	@Override
	public double getDtRightSpeed() {
		return dtRight.get();
	}

	/**
	 * Updates the PIDControllers' PIDConstants based on SmartDashboard values
	 */
	@Override
	public void updatePidConstants() {
		leftVelocityController.setPID(Robot.getConst("VelocityLeftkP", 1), Robot.getConst("VelocityLeftkI", 0),
				Robot.getConst("VelocityLeftkD", 0));
		rightVelocityController.setPID(Robot.getConst("VelocityRightkP", 1), Robot.getConst("VelocityRightkI", 0),
				Robot.getConst("VelocityRightkD", 0));
		resetVelocityPIDkFConsts();
	}

	/**
	 * Enable the VelocityPIDControllers used for velocity control on each side of
	 * the DT
	 */
	@Override
	public void enableVelocityPIDs() {
		leftVelocityController.enable();
		rightVelocityController.enable();
	}

	/**
	 * Disables the VelocityPIDControllers used for velocity control on each side of
	 * the DT
	 */
	@Override
	public void disableVelocityPIDs() {
		leftVelocityController.disable();
		rightVelocityController.disable();
	}

	/**
	 * Resets the AHRS value
	 */
	@Override
	public void resetAHRS() {
		fancyGyro.reset();
	}

	/**
	 * Used to get the yaw angle (Z-axis in degrees) that the ahrs currently reads
	 * 
	 * @return The angle that the ahrs reads (in degrees)
	 */
	@Override
	public double getAHRSAngle() {
		return fancyGyro.getAngle();
	}

	/**
	 * Resets the encoders' distances to zero
	 */
	@Override
	public void resetDistEncs() {
		leftEncDist.reset();
		rightEncDist.reset();
	}

	/**
	 * Sets the distancePerPulse property on the left encoder
	 * 
	 * @param ratio
	 *            The ratio to set the distancePerPulse to (real dist units/encoder
	 *            pulses)
	 */
	@Override
	public void setDistancePerPulseLeft(double ratio) {
		leftEncDist.setDistancePerPulse(ratio);
	}

	/**
	 * Sets the distancePerPulse property on the right encoder
	 * 
	 * @param ratio
	 *            The ratio to set the distancePerPulse to (real dist units/encoder
	 *            pulses)
	 */
	@Override
	public void setDistancePerPulseRight(double ratio) {
		rightEncDist.setDistancePerPulse(ratio);
	}

	/**
	 * Returns the distance (in real units) that the left encoder reads
	 * 
	 * @return How far the left encoder has traveled in real units since last reset
	 */
	@Override
	public double getLeftEncDist() {
		return leftEncDist.getDistance();
	}

	/**
	 * Returns the distance (in real units) that the right encoder reads
	 * 
	 * @return How far the right encoder has traveled in real units since last reset
	 */
	@Override
	public double getRightEncDist() {
		return rightEncDist.getDistance();
	}

	/**
	 * Activates the solenoid to push the drivetrain into high or low gear.
	 * 
	 * @param highGear
	 *            If the solenoid is to be pushed into high gear (true, kForward) or
	 *            low gear (false, kReverse)
	 */
	@Override
	public void shiftGears(boolean highGear) {
		if (highGear ^ Robot.getBool("Drivetrain Gear Shift Low", false)) {
			dtGear.set(DoubleSolenoid.Value.kForward);
		} else {
			dtGear.set(DoubleSolenoid.Value.kReverse);
		}
	}

	/**
	 * Stops the solenoid that pushes the drivetrain into low or high gear
	 */
	@Override
	public void shiftGearSolenoidOff() {
		dtGear.set(DoubleSolenoid.Value.kOff);
	}

	/**
	 * Reset the kf constants for both VelocityPIDControllers based on current DT
	 * gearing (high or low gear).
	 * 
	 * @param newKF
	 *            the new kF constant based on high and low gear max speeds; should
	 *            be 1 / max speed
	 * @return the new kF value as 1 / correct max speed
	 */
	@Override
	public double resetVelocityPIDkFConsts() {
		double newKF = 1 / getCurrentMaxSpeed();
		leftVelocityController.setF(newKF);
		rightVelocityController.setF(newKF);
		SmartDashboard.putNumber("VPID kF", newKF);
		return newKF;
	}
	
	public double resetVPIDInputRanges() {
		double currentMaxSpd = getCurrentMaxSpeed();
		leftVelocityController.setInputRange(-currentMaxSpd, currentMaxSpd);
		rightVelocityController.setInputRange(-currentMaxSpd, currentMaxSpd);
		return currentMaxSpd;
	}

	/**
	 * Gets the current max speed of the DT based on gearing (high or low gear)
	 * 
	 * @return the current max speed of the DT in inches/second
	 */
	@Override
	public double getCurrentMaxSpeed() {
		if (Robot.getBool("High Gear", true)) {
			return Robot.getConst("Max High Speed", 204);
		} else {
			return Robot.getConst("Max Low Speed", 84);
		}
	}
}
