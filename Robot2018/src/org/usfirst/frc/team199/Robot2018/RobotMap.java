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

	public static WPI_TalonSRX intakeMotor;
	public static WPI_TalonSRX liftMotor;
	public static WPI_TalonSRX climberMotor;

	public static DigitalSource leftEncPort1;
	public static DigitalSource leftEncPort2;
	public static Encoder leftEncDist;
	public static Encoder leftEncRate;
	public static WPI_TalonSRX dtLeftMaster;
	public static WPI_VictorSPX dtLeftSlave;
	public static SpeedControllerGroup dtLeft;
	public static VelocityPIDController leftVelocityController;

	public static DigitalSource rightEncPort1;
	public static DigitalSource rightEncPort2;
	public static Encoder rightEncDist;
	public static Encoder rightEncRate;
	public static WPI_TalonSRX dtRightMaster;
	public static WPI_VictorSPX dtRightSlave;
	public static SpeedControllerGroup dtRight;
	public static VelocityPIDController rightVelocityController;

	public static DifferentialDrive robotDrive;
	public static PIDSourceAverage distEncAvg;

	public static AHRS fancyGyro;
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

		intakeMotor = new WPI_TalonSRX(getPort("IntakeTalonSRX", 4));
		configSRX(intakeMotor);
		liftMotor = new WPI_TalonSRX(getPort("LiftTalonSRX", 5));
		configSRX(liftMotor);
		climberMotor = new WPI_TalonSRX(getPort("ClimberTalonSRX", 6));
		configSRX(climberMotor);

		leftEncPort1 = new DigitalInput(getPort("1LeftEnc", 0));
		leftEncPort2 = new DigitalInput(getPort("2LeftEnc", 1));
		leftEncDist = new Encoder(leftEncPort1, leftEncPort2);
		leftEncDist.setPIDSourceType(PIDSourceType.kDisplacement);
		leftEncRate = new Encoder(leftEncPort1, leftEncPort2);
		leftEncRate.setPIDSourceType(PIDSourceType.kRate);
		dtLeftMaster = new WPI_TalonSRX(getPort("LeftTalonSRXMaster", 0));
		configSRX(dtLeftMaster);
		dtLeftSlave = new WPI_VictorSPX(getPort("LeftTalonSPXSlave", 1));
		configSPX(dtLeftSlave);
		dtLeft = new SpeedControllerGroup(dtLeftMaster, dtLeftSlave);

		leftVelocityController = new VelocityPIDController(Robot.getConst("VelocityLeftkP", 1),
				Robot.getConst("VelocityLeftkI", 0), Robot.getConst("VelocityLeftkD", calcDefkD()),
				1 / Robot.getConst("Max Low Speed", 84), leftEncRate, dtLeft);
		leftVelocityController.enable();
		leftVelocityController.setInputRange(-Robot.getConst("Max High Speed", 204),
				Robot.getConst("Max High Speed", 204));
		leftVelocityController.setOutputRange(-1.0, 1.0);
		leftVelocityController.setContinuous(false);
		leftVelocityController.setAbsoluteTolerance(Robot.getConst("VelocityToleranceLeft", 2));

		rightEncPort1 = new DigitalInput(getPort("1RightEnc", 2));
		rightEncPort2 = new DigitalInput(getPort("2RightEnc", 3));
		rightEncDist = new Encoder(leftEncPort1, leftEncPort2);
		rightEncDist.setPIDSourceType(PIDSourceType.kDisplacement);
		rightEncRate = new Encoder(leftEncPort1, leftEncPort2);
		rightEncRate.setPIDSourceType(PIDSourceType.kRate);
		dtRightMaster = new WPI_TalonSRX(getPort("RightTalonSRXMaster", 2));
		configSRX(dtRightMaster);
		dtRightSlave = new WPI_VictorSPX(getPort("RightTalonSPXSlave", 3));
		configSPX(dtRightSlave);
		dtRight = new SpeedControllerGroup(dtRightMaster, dtRightSlave);

		rightVelocityController = new VelocityPIDController(Robot.getConst("VelocityRightkP", 1),
				Robot.getConst("VelocityRightkI", 0), Robot.getConst("VelocityRightkD", calcDefkD()),
				1 / Robot.getConst("Max Low Speed", 84), rightEncRate, dtRight);
		rightVelocityController.enable();
		rightVelocityController.setInputRange(-Robot.getConst("Max High Speed", 204),
				Robot.getConst("Max High Speed", 204));
		rightVelocityController.setOutputRange(-1.0, 1.0);
		rightVelocityController.setContinuous(false);
		rightVelocityController.setAbsoluteTolerance(Robot.getConst("VelocityToleranceRight", 2));

		robotDrive = new DifferentialDrive(leftVelocityController, rightVelocityController);
		robotDrive.setMaxOutput(Robot.getConst("Max High Speed", 204));

		distEncAvg = new PIDSourceAverage(leftEncDist, rightEncDist);
		fancyGyro = new AHRS(SerialPort.Port.kMXP);
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

	/**
	 * Uses SmartDashboard and math to calculate a *great* default kD
	 */
	public double calcDefkD() {
		double timeConstant = Robot.getConst("Omega Max", 5330) * Robot.getConst("Mass of Robot", 54.4311)
				* Robot.getConst("Radius of Drivetrain Wheel", 0.0635)
				* Robot.getConst("Radius of Drivetrain Wheel", 0.0635) / Robot.getConst("Stall Torque", 2.41);
		double cycleTime = Robot.getConst("Code cycle time", 0.1);
		double denominator = 1 - Math.pow(Math.E, -1 * cycleTime / timeConstant);
		return 1 / denominator;
	}
}
