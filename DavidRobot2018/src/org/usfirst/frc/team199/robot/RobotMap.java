/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
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
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	public static Encoder leftEnc;
	public static WPI_TalonSRX dtLeftDrive;
		public static WPI_TalonSRX dtLeftSlave;
	public static SpeedControllerGroup dtLeft;
		
	public static Encoder rightEnc;
	public static WPI_TalonSRX dtRightDrive;
		public static WPI_TalonSRX dtRightSlave;
	public static SpeedControllerGroup dtRight;
	public static DifferentialDrive robotDrive;
	public static PIDController turnController;
	public static PIDController moveController;
//	public static PIDController moveLeftController;
//	public static PIDController moveRightController;
	
	public static AHRS ahrs;
	public static AnalogGyro dtGyro;
	public static DoubleSolenoid dtGear;
	
	private void configSRX(WPI_TalonSRX mc) {
		int kTimeout = (int) Robot.getConst("ConstkTimeoutMs", 10);
		mc.configNominalOutputForward(0, kTimeout);
        mc.configNominalOutputReverse(0, kTimeout);
        mc.configPeakOutputForward(1, kTimeout);
        mc.configPeakOutputReverse(-1, kTimeout);
	}
	public RobotMap() {
		
		leftEnc = new Encoder(getPort("Port1LeftEnc", 0), getPort("Port2LeftEnc", 1));
		dtLeftDrive = new WPI_TalonSRX(getPort("PortLeftTalonSRXDrive", 1));
		configSRX(dtLeftDrive);
			dtLeftSlave = new WPI_TalonSRX(getPort("PortLeftTalonSRXSlave", 1));
			configSRX(dtLeftSlave);
		dtLeft = new SpeedControllerGroup(dtLeftDrive, dtLeftSlave);
		
		rightEnc = new Encoder(getPort("Port1RightEnc", 2), getPort("Port2RightEnc", 3));
		dtRightDrive = new WPI_TalonSRX(getPort("PortRightTalonSRXDrive", 2));
		configSRX(dtRightDrive);
			dtRightSlave = new WPI_TalonSRX(getPort("PortRightTalonSRXSlave", 1));
			configSRX(dtRightSlave);
		dtRight = new SpeedControllerGroup(dtRightDrive, dtRightSlave);
		
		robotDrive = new DifferentialDrive(dtLeft, dtRight);
		turnController = new PIDController(Robot.getConst("ConstTurnkP", 1), Robot.getConst("ConstTurnkI", 0), Robot.getConst("ConstTurnkD", 0), ahrs, Robot.dt);
			turnController.disable(); 
			turnController.setInputRange(-180, 180);
			turnController.setOutputRange(-1.0, 1.0);
			turnController.setContinuous();
			turnController.setAbsoluteTolerance(Robot.getConst("ConstTurnTolerance", 1));
		moveController = new PIDController(Robot.getConst("ConstMovekP", 1), Robot.getConst("ConstMovekI", 0), Robot.getConst("ConstMovekD", 0), Robot.dt, Robot.dt);
			turnController.disable(); 
			turnController.setInputRange(-180, 180);
			turnController.setOutputRange(-1.0, 1.0);
			turnController.setContinuous();
			turnController.setAbsoluteTolerance(Robot.getConst("ConstMoveTolerance", 2));
//		moveLeftController = new PIDController(Robot.getConst("ConstMoveLeftkP", 1), Robot.getConst("ConstMoveLeftkI", 0), Robot.getConst("ConstMoveLeftkD", 0), Robot.dt.getLeftDrive(), (PIDOutput) Robot.dt.getLeftDrive());
//			moveLeftController.disable();
//			moveLeftController.setInputRange(0, Double.MAX_VALUE);
//			moveLeftController.setOutputRange(-1.0, 1.0);
//			moveLeftController.setContinuous(false);
//			moveLeftController.setAbsoluteTolerance(Robot.getConst("ConstMoveToleranceLeft", 2));
//		moveRightController = new PIDController(Robot.getConst("ConstMoveRightkP", 1), Robot.getConst("ConstMoveRightkI", 0), Robot.getConst("ConstMoveRightkD", 0), Robot.dt.getRightDrive(), (PIDOutput) Robot.dt.getRightDrive());
//			moveRightController.disable();
//			moveRightController.setInputRange(0, Double.MAX_VALUE);
//			moveRightController.setOutputRange(-1.0, 1.0);
//			moveRightController.setContinuous(false);
//			moveRightController.setAbsoluteTolerance(Robot.getConst("ConstMoveToleranceRight", 2));
			
		ahrs = new AHRS(SerialPort.Port.kMXP);
		dtGyro = new AnalogGyro(getPort("PortGyro", 0));
		dtGear = new DoubleSolenoid(getPort("Port1dtGearSolenoid", 0), getPort("Port2dtGearSolenoid", 1));
		
	}
	
	/**
	 * Used in RobotMap to find ports for robot components, getPort also puts numbers if they don't exist yet.
	 * @param key The port key
	 * @param def The default value
	 * @return returns the Port
	 */
	public int getPort(String key, int def) {
		if(!SmartDashboard.containsKey(key)) {
			SmartDashboard.putNumber(key, def);
		}
		return (int)SmartDashboard.getNumber(key, def);
	}
}
