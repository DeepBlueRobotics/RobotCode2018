/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018;

import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;
import org.usfirst.frc.team199.Robot2018.autonomous.VelocityPIDController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static DigitalSource leftEncPort1;
	public static DigitalSource leftEncPort2;
	public static Encoder leftEncDist;
	public static Encoder leftEncRate;
	public static WPI_TalonSRX dtLeftDrive;
	public static WPI_VictorSPX dtLeftSlave;
	public static SpeedControllerGroup dtLeft;
	public static VelocityPIDController leftVelocityController;

	public static DigitalSource rightEncPort1;
	public static DigitalSource rightEncPort2;
	public static Encoder rightEncDist;
	public static Encoder rightEncRate;
	public static WPI_TalonSRX dtRightDrive;
	public static WPI_VictorSPX dtRightSlave;
	public static SpeedControllerGroup dtRight;
	public static VelocityPIDController rightVelocityController;

	public static DifferentialDrive robotDrive;
	public static PIDSourceAverage distEncAvg;

	public static AHRS fancyGyro;
	public static AnalogGyro dtGyro;
	public static DoubleSolenoid dtGear;

	/**
	 * This function takes in a TalonSRX motorController and sets nominal and peak
	 * outputs to the default
	 * 
	 * @param mc
	 *            the TalonSRX to configure
	 */
	private void configSRX(WPI_TalonSRX mc) {
		int kTimeout = (int) Robot.getConst("kTimeoutMs", 10);
		mc.configNominalOutputForward(0, kTimeout);
		mc.configNominalOutputReverse(0, kTimeout);
		mc.configPeakOutputForward(1, kTimeout);
		mc.configPeakOutputReverse(-1, kTimeout);
	}

	/**
	 * This function takes in a VictorSPX motorController and sets nominal and peak
	 * outputs to the default
	 * 
	 * @param mc
	 *            the VictorSPX to configure
	 */
	private void configSPX(WPI_VictorSPX mc) {
		int kTimeout = (int) Robot.getConst("kTimeoutMs", 10);
		mc.configNominalOutputForward(0, kTimeout);
		mc.configNominalOutputReverse(0, kTimeout);
		mc.configPeakOutputForward(1, kTimeout);
		mc.configPeakOutputReverse(-1, kTimeout);
	}

	public RobotMap() {

		leftEncPort1 = new DigitalInput(getPort("1LeftEnc", 0));
		leftEncPort2 = new DigitalInput(getPort("2LeftEnc", 1));
		leftEncDist = new Encoder(leftEncPort1, leftEncPort2);
		leftEncDist.setPIDSourceType(PIDSourceType.kDisplacement);
		leftEncRate = new Encoder(leftEncPort1, leftEncPort2);
		leftEncRate.setPIDSourceType(PIDSourceType.kRate);
		dtLeftDrive = new WPI_TalonSRX(getPort("LeftTalonSRXDrive", 0));
		configSRX(dtLeftDrive);
		dtLeftSlave = new WPI_VictorSPX(getPort("LeftTalonSPXSlave", 1));
		configSPX(dtLeftSlave);
		dtLeft = new SpeedControllerGroup(dtLeftDrive, dtLeftSlave);

		leftVelocityController = new VelocityPIDController(Robot.getConst("MoveLeftkP", 1),
				Robot.getConst("MoveLeftkI", 0), Robot.getConst("MoveLeftkD", 0), leftEncRate, dtLeft);
		leftVelocityController.enable();
		leftVelocityController.setInputRange(0, Double.MAX_VALUE);
		leftVelocityController.setOutputRange(-1.0, 1.0);
		leftVelocityController.setContinuous(false);
		leftVelocityController.setAbsoluteTolerance(Robot.getConst("ConstMoveToleranceLeft", 2));

		rightEncPort1 = new DigitalInput(getPort("1RightEnc", 2));
		rightEncPort2 = new DigitalInput(getPort("2RightEnc", 3));
		rightEncDist = new Encoder(leftEncPort1, leftEncPort2);
		rightEncDist.setPIDSourceType(PIDSourceType.kDisplacement);
		rightEncRate = new Encoder(leftEncPort1, leftEncPort2);
		rightEncRate.setPIDSourceType(PIDSourceType.kRate);
		dtRightDrive = new WPI_TalonSRX(getPort("RightTalonSRXDrive", 2));
		configSRX(dtRightDrive);
		dtRightSlave = new WPI_VictorSPX(getPort("RightTalonSPXSlave", 3));
		configSPX(dtRightSlave);
		dtRight = new SpeedControllerGroup(dtRightDrive, dtRightSlave);

		rightVelocityController = new VelocityPIDController(Robot.getConst("ConstMoveRightkP", 1),
				Robot.getConst("ConstMoveRightkI", 0), Robot.getConst("ConstMoveRightkD", 0), rightEncRate, dtRight);
		rightVelocityController.enable();
		rightVelocityController.setInputRange(0, Double.MAX_VALUE);
		rightVelocityController.setOutputRange(-1.0, 1.0);
		rightVelocityController.setContinuous(false);
		rightVelocityController.setAbsoluteTolerance(Robot.getConst("ConstMoveToleranceRight", 2));

		robotDrive = new DifferentialDrive(leftVelocityController, rightVelocityController);

		distEncAvg = new PIDSourceAverage(leftEncDist, rightEncDist);
		fancyGyro = new AHRS(SerialPort.Port.kMXP);
		dtGyro = new AnalogGyro(getPort("Gyro", 0));
		dtGear = new DoubleSolenoid(getPort("1dtGearSolenoid", 0), getPort("2dtGearSolenoid", 1));
	}

	/**
	 * Used in RobotMap to find ports for robot components, getPort also puts
	 * numbers if they don't exist yet.
	 * 
	 * @param key
	 *            The port key
	 * @param def
	 *            The default value
	 * @return returns the Port
	 */
	public int getPort(String key, int def) {
		if (!SmartDashboard.containsKey("Port/" + key)) {
			SmartDashboard.putNumber("Port/" + key, def);
		}
		return (int) SmartDashboard.getNumber("Port/" + key, def);
	}
}
