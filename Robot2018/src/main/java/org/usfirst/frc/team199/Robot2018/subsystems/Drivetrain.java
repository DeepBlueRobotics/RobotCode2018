/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;
import org.usfirst.frc.team199.Robot2018.autonomous.VelocityPIDController;
import org.usfirst.frc.team199.Robot2018.commands.TeleopDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem implements DrivetrainInterface {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final WPI_TalonSRX dtLeftMaster = RobotMap.dtLeftMaster;
	private final WPI_VictorSPX dtLeftSlave = RobotMap.dtLeftSlave;
	private final WPI_TalonSRX dtRightMaster = RobotMap.dtRightMaster;
	private final WPI_VictorSPX dtRightSlave = RobotMap.dtRightSlave;
	private final Encoder leftEncDist = RobotMap.leftEncDist;
	private final Encoder rightEncDist = RobotMap.rightEncDist;
	private final Encoder leftEncRate = RobotMap.leftEncRate;
	private final Encoder rightEncRate = RobotMap.rightEncRate;
	private final PIDSourceAverage distEncAvg = RobotMap.distEncAvg;
	public final SpeedControllerGroup dtLeft = RobotMap.dtLeft;
	public final SpeedControllerGroup dtRight = RobotMap.dtRight;
	private DifferentialDrive robotDrive;
	private VelocityPIDController leftVelocityController;
	private VelocityPIDController rightVelocityController;

	private final AHRS fancyGyro = RobotMap.fancyGyro;
	private final DoubleSolenoid dtGear = RobotMap.dtGear;

	private SmartDashboardInterface sd;

	private boolean highGear;
	private boolean isInverted;
	private int inverted;

	/**
	 * Sets up velocity PID controllers. Initializes to not in high gear and to not
	 * inverted.
	 */
	public Drivetrain(SmartDashboardInterface sd) {
		this.sd = sd;

		highGear = false;

		isInverted = false;
		inverted = 1;

		// all 0s for controller construction because they all get set to right values
		// by resetAllVelocityPIDConsts
		leftVelocityController = new VelocityPIDController(0, 0, 0, 0, leftEncRate, dtLeft);
		rightVelocityController = new VelocityPIDController(0, 0, 0, 0, rightEncRate, dtRight);
		resetAllVelocityPIDConsts();

		leftVelocityController.setInputRange(-Robot.getConst("Max High Speed", 204),
				Robot.getConst("Max High Speed", 204));
		leftVelocityController.setOutputRange(-1.0, 1.0);
		leftVelocityController.setContinuous(false);
		leftVelocityController.setAbsoluteTolerance(Robot.getConst("VelocityToleranceLeft", 2));
		SmartDashboard.putData(leftVelocityController);

		rightVelocityController.setInputRange(-Robot.getConst("Max High Speed", 204),
				Robot.getConst("Max High Speed", 204));
		rightVelocityController.setOutputRange(-1.0, 1.0);
		rightVelocityController.setContinuous(false);
		rightVelocityController.setAbsoluteTolerance(Robot.getConst("VelocityToleranceRight", 2));

		if (Robot.getBool("Teleop velocity PID", false)) {
			robotDrive = new DifferentialDrive(leftVelocityController, rightVelocityController);
			robotDrive.setMaxOutput(Robot.getConst("Max High Speed", 204));
		} else {
			robotDrive = new DifferentialDrive(dtLeft, dtRight);
		}
	}

	/**
	 * Inverts the drivetrain (forward is backward, left is right) by toggling a
	 * motor output multiplier (between 1 and -1) and the distance encoders (between
	 * inverted or not). The motor output multiplier is used in arcadeDrive and
	 * tankDrive for final speed/turn and left/right outputs.
	 */
	public void reverseDT() {
		inverted *= -1;
		isInverted = !isInverted;
		leftEncDist.setReverseDirection(isInverted);
		rightEncDist.setReverseDirection(isInverted);
		SmartDashboard.putBoolean("DT is Inverted", isInverted);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}

	@Override
	public boolean isVPIDUsed() {
		return Robot.getBool("Teleop velocity PID", false);
	}

	public PIDSourceAverage getDistEncAvg() {
		return distEncAvg;
	}

	public double getEncAvgDist() {
		return distEncAvg.pidGet();
	}

	public double getLeftDist() {
		return leftEncDist.getDistance();
	}

	public double getRightDist() {
		return rightEncDist.getDistance();
	}

	public void setLeft(double spd) {
		dtLeft.set(spd);
	}

	public void setRight(double spd) {
		dtRight.set(spd);
	}

	/**
	 * Use for testing only (i.e. when not going through robotDrive)
	 */
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

	public double getLeftVPIDOutput() {
		return dtLeft.get();
	}

	public double getRightVPIDOutput() {
		return dtRight.get();
	}

	public boolean VPIDsOnTarg() {
		return leftVelocityController.onTarget() && rightVelocityController.onTarget();
	}

	/**
	 * Returns the getRate() of the left encoder
	 * 
	 * @return the rate of the left encoder
	 */
	@Override
	public double getLeftEncRate() {
		return leftEncRate.getRate();
	}

	/**
	 * Returns the getRate() of the right encoder
	 * 
	 * @return the rate of the right encoder
	 */
	@Override
	public double getRightEncRate() {
		return rightEncRate.getRate();
	}

	/**
	 * Sets the left side of the drivetrain to use the talons' pids to run at the
	 * specified speed.
	 * 
	 * @param value
	 *            The speed to run at
	 */
	public void dtLeftPIDDrive(double value) {
		double setValue = value * Robot.getConst("Units per 100ms", 3413);
		dtLeftMaster.set(ControlMode.Velocity, setValue);
		dtLeftSlave.set(ControlMode.Velocity, setValue);
	}

	/**
	 * Sets the right side of the drivetrain to use the talons' pids to run at the
	 * specified speed.
	 * 
	 * @param value
	 *            The speed to run at
	 */
	public void dtRightPIDDrive(double value) {
		double setValue = value * Robot.getConst("Units per 100ms", 3413);
		dtRightMaster.set(ControlMode.Velocity, setValue);
		dtRightSlave.set(ControlMode.Velocity, setValue);
	}

	public VelocityPIDController getLeftVPID() {
		return leftVelocityController;
	}

	public VelocityPIDController getRightVPID() {
		return rightVelocityController;
	}

	/**
	 * Drives based on joystick input and SmartDashboard values
	 */
	@Override
	public void teleopDrive() {
		boolean squareJoy = Robot.getBool("Square Joystick Values", true);
		if (SmartDashboard.getBoolean("Arcade Drive", true)) {
			double forw;
			double turn;
			if (Robot.getBool("Arcade Drive Default Setup", true)) {
				forw = -Robot.oi.leftJoy.getY();
				turn = Robot.oi.rightJoy.getX();
				System.out.println("Forward is " + forw + " and turn is " + turn);
				Robot.dt.arcadeDrive(squareJoy ? Robot.oi.squareValueKeepSign(forw) : forw,
						squareJoy ? Robot.oi.squareValueKeepSign(turn) : turn);
			} else {
				forw = -Robot.oi.rightJoy.getY();
				turn = Robot.oi.leftJoy.getX();
				Robot.dt.arcadeDrive(squareJoy ? Robot.oi.squareValueKeepSign(forw) : forw,
						squareJoy ? Robot.oi.squareValueKeepSign(turn) : turn);
			}
		} else {
			double left = -Robot.oi.leftJoy.getY();
			double right = -Robot.oi.rightJoy.getY();
			Robot.dt.tankDrive(squareJoy ? Robot.oi.squareValueKeepSign(left) : left,
					squareJoy ? Robot.oi.squareValueKeepSign(right) : right);
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
		if (Robot.getBool("Arcade Drive Default Setup", true)) {
			if (Robot.oi.leftJoy.getRawButton(1)) {
				speed *= Robot.getConst("Speed Slow Ratio", 0.5);
			}
			if (Robot.oi.rightJoy.getRawButton(1)) {
				turn *= Robot.getConst("Turn Slow Ratio", 0.5);
			}
		} else {
			if (Robot.oi.rightJoy.getRawButton(1)) {
				speed *= Robot.getConst("Speed Slow Ratio", 0.5);
			}
			if (Robot.oi.leftJoy.getRawButton(1)) {
				turn *= Robot.getConst("Turn Slow Ratio", 0.5);
			}
		}
		robotDrive.arcadeDrive(inverted * speed, turn, Robot.getBool("Square Drive Values", false));
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
		if (Robot.oi.leftJoy.getRawButton(1)) {
			leftSpeed *= Robot.getConst("Speed Slow Ratio", 0.5);
		}
		if (Robot.oi.rightJoy.getRawButton(1)) {
			rightSpeed *= Robot.getConst("Speed Slow Ratio", 0.5);
		}
		if (isInverted) {
			double leftSpd = leftSpeed;
			leftSpeed = rightSpeed;
			rightSpeed = leftSpd;
		}
		robotDrive.tankDrive(inverted * leftSpeed, inverted * rightSpeed, Robot.getBool("Square Drive Values", false));
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
		// reset disables and also clears the error so that it isn't used when they
		// are reenabled.
		leftVelocityController.reset();
		rightVelocityController.reset();
	}

	/**
	 * Resets the AHRS value
	 */
	@Override
	public void resetAHRS() {
		fancyGyro.reset();
	}

	@Override
	public double getGyroRate() {
		return fancyGyro.getRate();
	}

	/**
	 * Used to get the yaw angle (Z-axis in degrees) that the ahrs currently reads
	 * 
	 * @return The angle that the ahrs reads (in degrees)
	 */
	@Override
	public double getAHRSAngle() {
		return fancyGyro.getYaw();
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
		leftEncRate.setDistancePerPulse(ratio);
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
		rightEncRate.setDistancePerPulse(ratio);
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
	 *            If the solenoid is to be pushed into high gear (true, kReverse) or
	 *            low gear (false, kForward)
	 */
	@Override
	public void shiftGears(boolean shiftToHighGear) {
		if (shiftToHighGear ^ Robot.getBool("Drivetrain Gear Shift Low", false)) {
			dtGear.set(DoubleSolenoid.Value.kReverse);
			highGear = true;
		} else {
			dtGear.set(DoubleSolenoid.Value.kForward);
			highGear = false;
		}
		sd.putBoolean("High Gear", highGear);
		resetAllVelocityPIDConsts();
		resetVPIDAndRobotDriveRanges();
	}

	/**
	 * Stops the solenoid that pushes the drivetrain into low or high gear
	 */
	@Override
	public void shiftGearSolenoidOff() {
		dtGear.set(DoubleSolenoid.Value.kOff);
	}

	/**
	 * @return the gyroscope
	 */
	@Override
	public PIDSource getGyro() {
		return fancyGyro;
	}

	@Override
	public void resetAllVelocityPIDConsts() {
		resetVelocityPIDkFConsts();
		resetVelocityPIDkPConsts();
		resetVelocityPIDkIConsts();
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
	public void resetVelocityPIDkFConsts() {
		// double newKF = 1 / getCurrentMaxSpeed();
		// for this way of doing velocity PID, kF should always be 0
		double newKF = 0;
		leftVelocityController.setF(newKF);
		rightVelocityController.setF(newKF);
		SmartDashboard.putNumber("VPID kF", newKF);
	}

	/**
	 * Reset the kI constants for both VelocityPIDControllers based on current DT
	 * gearing (high or low gear).
	 */
	@Override
	public void resetVelocityPIDkIConsts() {
		// 0.011 was calculated manually. 84 is the low gear max speed, to which we
		// scale the constants.
		double defaultkI = Robot.getConst("VelocityPidR", 3) / Robot.rmap.getDrivetrainTimeConstant()
				* Robot.rmap.getCycleTime();
		double newLeftkI = Robot.getConst("VelocityLeftkI", defaultkI) / getCurrentMaxSpeed();
		double newRightkI = Robot.getConst("VelocityRightkI", defaultkI) / getCurrentMaxSpeed();
		// I is P because wpilib is dumb
		leftVelocityController.setP(newLeftkI);
		rightVelocityController.setP(newRightkI);
		SmartDashboard.putNumber("VPID Left kI", newLeftkI);
		SmartDashboard.putNumber("VPID Right kI", newRightkI);
	}

	/**
	 * Reset the kD constants for both VelocityPIDControllers based on current DT
	 * gearing (high or low gear).
	 */
	@Override
	public void resetVelocityPIDkPConsts() {
		// 0.012 was calculated manually. 84 is the low gear max speed, to which we
		// scale the constants.
		double newkP = Robot.getConst("VelocityPidR", 3) / getCurrentMaxSpeed();
		// P is D because wpilib is dumb
		leftVelocityController.setD(newkP);
		rightVelocityController.setD(newkP);
		SmartDashboard.putNumber("VPID Left kP", newkP);
		SmartDashboard.putNumber("VPID Right kP", newkP);
	}

	public double resetVPIDAndRobotDriveRanges() {
		double currentMaxSpd = getCurrentMaxSpeed();
		leftVelocityController.setInputRange(-currentMaxSpd, currentMaxSpd);
		rightVelocityController.setInputRange(-currentMaxSpd, currentMaxSpd);
		if (Robot.getBool("Teleop velocity PID", false)) {
			robotDrive.setMaxOutput(currentMaxSpd);
		}
		return currentMaxSpd;
	}

	/**
	 * Gets the current max speed of the DT based on gearing (high or low gear)
	 * 
	 * @return the current max speed of the DT in inches/second
	 */
	@Override
	public double getCurrentMaxSpeed() {
		if (highGear) {
			return Robot.getConst("Max High Speed", 204);
		} else {
			return Robot.getConst("Max Low Speed", 84);
		}
	}

	/**
	 * Put left and right velocity controllers (PID) on SmartDashboard.
	 */
	@Override
	public void putVelocityControllersToDashboard() {
		SmartDashboard.putData("Left PID Controller", leftVelocityController);
		SmartDashboard.putData("Right PID Controller", rightVelocityController);
	}

	@Override
	public double getPIDMoveConstant() {
		double G = Robot.rmap.getGearRatio();
		double T = Robot.rmap.getStallTorque();
		double fudge = Robot.getConst("PID Move Fudge Factor", 0.25);
		T *= fudge;
		double R = Robot.rmap.getRadius();
		double M = Robot.rmap.getWeight();
		M = convertNtokG(M);
		return Math.sqrt(Robot.rmap.convertMtoIn((8 * T * G) / (R * M)));
	}

	@Override
	public double getPIDTurnConstant() {
		double G = Robot.rmap.getGearRatio();
		double T = Robot.rmap.getStallTorque();
		double R = Robot.rmap.getRadius();
		double M = Robot.rmap.getWeight();
		M = convertNtokG(M);
		return 4 * Math.sqrt(Robot.rmap.convertMtoIn((T * G) / (R * M)));
	}

	private double convertNtokG(double newtons) {
		// weight / accel due to grav = kg
		return newtons / 9.81;
	}
}
