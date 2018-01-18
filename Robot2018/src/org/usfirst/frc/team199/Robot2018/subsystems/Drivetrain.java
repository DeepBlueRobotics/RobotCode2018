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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem implements PIDOutput, PIDSource {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final Encoder leftEnc = RobotMap.leftEnc;
	private final Encoder rightEnc = RobotMap.rightEnc;
	private final WPI_TalonSRX dtLeftDrive = RobotMap.dtLeftDrive;
	private final WPI_TalonSRX dtRightDrive = RobotMap.dtRightDrive;
	private final SpeedControllerGroup dtLeft = RobotMap.dtLeft;
	private final SpeedControllerGroup dtRight = RobotMap.dtRight;
	private final DifferentialDrive robotDrive = RobotMap.robotDrive;
	private final PIDController turnController = RobotMap.turnController;
	private final PIDController moveController = RobotMap.moveController;
	// private final PIDController moveLeftController = RobotMap.moveLeftController;
	// private final PIDController moveRightController =
	// RobotMap.moveRightController;

	private final AHRS ahrs = RobotMap.ahrs;
	private final AnalogGyro dtGyro = RobotMap.dtGyro;
	private final DoubleSolenoid dtGear = RobotMap.dtGear;

	public void pushGear(boolean forw) {
		if (forw ^ Robot.getBool("Drivetrain Gear Shift Backwards", false)) {
			dtGear.set(DoubleSolenoid.Value.kForward);
		} else {
			dtGear.set(DoubleSolenoid.Value.kReverse);
		}
	}

	public void stopGear() {
		dtGear.set(DoubleSolenoid.Value.kOff);
	}

	private double pidOut = 0;

	public void resetAHRS() {
		ahrs.reset();
	}

	public void setLeftMotor(double value) {
		dtLeft.set(value);
	}

	private void stopLeftMotor() {
		dtLeft.stopMotor();
	}

	public void setRightMotor(double value) {
		dtRight.set(value);
	}

	private void stopRightMotor() {
		dtRight.stopMotor();
	}

	public void stopDrive() {
		stopRightMotor();
		stopLeftMotor();
	}

	public void resetEnc() {
		leftEnc.reset();
		rightEnc.reset();
	}

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

	public void arcadeDrive(double speed, double turn) {
		robotDrive.arcadeDrive(speed, turn, Robot.getBool("Square Drive Values", false));
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		robotDrive.tankDrive(leftSpeed, rightSpeed, Robot.getBool("Square Drive Values", false));
	}

	public double getDtLeft() {
		return dtLeft.get();
	}

	public double getDtRight() {
		return dtRight.get();
	}

	public double getGyro() {
		return dtGyro.getAngle();
	}

	public void resetGyro() {
		dtGyro.reset();
	}

	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}

	public void disableTurnPid() {
		turnController.disable();
	}

	public void enableTurnPid() {
		turnController.enable();
	}

	public void setSetTurn(double set) {
		turnController.setSetpoint(set);
	}

	// public void disableMoveLeftPid() {
	// moveLeftController.disable();
	// }
	// public void enableMoveLeftPid() {
	// moveLeftController.enable();
	// }
	// public void setSetMoveLeft(double set) {
	// moveLeftController.setSetpoint(set);
	// }
	//
	// public void disableMoveRightPid() {
	// moveRightController.disable();
	// }
	// public void enableMoveRightPid() {
	// moveRightController.enable();
	// }
	// public void setSetMoveRight(double set) {
	// moveRightController.setSetpoint(set);
	// }

	public void setDistancePerPulseLeft(double dist) {
		leftEnc.setDistancePerPulse(dist);
	}

	public void setDistancePerPulseRight(double dist) {
		rightEnc.setDistancePerPulse(dist);
	}

	public double getPidOut() {
		return pidOut;
	}

	@Override
	public double pidGet() {
		return average(leftEnc.getDistance(), rightEnc.getDistance());
	}

	public PIDSource getLeftDrive() {
		return (PIDSource) dtLeftDrive;
	}

	public PIDSource getRightDrive() {
		return (PIDSource) dtRightDrive;
	}

	// public boolean onTargetLeft() {
	// return moveLeftController.onTarget();
	// }
	// public boolean onTargetRight() {
	// return moveRightController.onTarget();
	// }
	public void enableMovePid() {
		moveController.enable();
	}

	public void disableMovePid() {
		moveController.disable();
	}

	public void setSetMove(double set) {
		moveController.setSetpoint(set);
	}

	@Override
	public void pidWrite(double output) {
		pidOut = output;
	}

	public static double average(double a, double b) {
		return (a + b) / 2;
	}

	public boolean onTurnTarg() {
		return turnController.onTarget();
	}

	public boolean onDriveTarg() {
		return moveController.onTarget();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}
}
