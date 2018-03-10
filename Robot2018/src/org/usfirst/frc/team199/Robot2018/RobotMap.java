/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018;

import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static PowerDistributionPanel pdp;

	public static WPI_TalonSRX liftMotor;
	public static Encoder liftEnc;
	public static DigitalSource liftEncPort1;
	public static DigitalSource liftEncPort2;

	public static WPI_TalonSRX climberMotor;

	public static VictorSP leftIntakeMotor;
	public static VictorSP rightIntakeMotor;
	public static DoubleSolenoid leftIntakeVerticalSolenoid;
	public static DoubleSolenoid rightIntakeVerticalSolenoid;
	public static DoubleSolenoid leftIntakeHorizontalSolenoid;
	public static DoubleSolenoid rightIntakeHorizontalSolenoid;

	public static DigitalSource leftEncPort1;
	public static DigitalSource leftEncPort2;
	public static Encoder leftEncDist;
	public static Encoder leftEncRate;
	public static WPI_TalonSRX dtLeftMaster;
	public static WPI_VictorSPX dtLeftSlave;
	public static SpeedControllerGroup dtLeft;

	public static DigitalSource rightEncPort1;
	public static DigitalSource rightEncPort2;
	public static Encoder rightEncDist;
	public static Encoder rightEncRate;
	public static WPI_TalonSRX dtRightMaster;
	public static WPI_VictorSPX dtRightSlave;
	public static SpeedControllerGroup dtRight;

	public static PIDSourceAverage distEncAvg;

	public static AHRS fancyGyro;
	public static DoubleSolenoid dtGear;

	private final double DIST_PER_PULSE_RATIO = (5.0 * Math.PI) * (17.0 / 25) / (3.0 * 256);

	/**
	 * This function takes in a TalonSRX motorController and sets nominal and peak
	 * outputs to the default
	 * 
	 * @param mc
	 *            the TalonSRX to configure
	 */
	private void configSRX(WPI_TalonSRX mc) {
		// stuff cole said to put
		int kTimeout = (int) Robot.getConst("kTimeoutMs", 10);
		mc.configNominalOutputForward(0, kTimeout);
		mc.configNominalOutputReverse(0, kTimeout);
		mc.configPeakOutputForward(1, kTimeout);
		mc.configPeakOutputReverse(-1, kTimeout);

		// current limiting stuff cole said to put
		mc.configPeakCurrentLimit(0, 0);
		mc.configPeakCurrentDuration(0, 0);
		// 40 amps is the amp limit of a CIM. also, the PDP has 40 amp circuit breakers,
		// so if we went above 40, the motors would stop completely
		mc.configContinuousCurrentLimit(40, 0);
		mc.enableCurrentLimit(true);

		mc.configNeutralDeadband(Robot.getConst("Motor Deadband", 0.001), kTimeout);
	}

	/**
	 * This function takes in a VictorSPX motorController and sets nominal and peak
	 * outputs to the default
	 * 
	 * @param mc
	 *            the VictorSPX to configure
	 */
	private void configSPX(WPI_VictorSPX mc) {
		// stuff cole said to put
		int kTimeout = (int) Robot.getConst("kTimeoutMs", 10);
		mc.configNominalOutputForward(0, kTimeout);
		mc.configNominalOutputReverse(0, kTimeout);
		mc.configPeakOutputForward(1, kTimeout);
		mc.configPeakOutputReverse(-1, kTimeout);

		mc.configNeutralDeadband(Robot.getConst("Motor Deadband", 0.001), kTimeout);
	}

	public RobotMap() {
		pdp = new PowerDistributionPanel();

		liftMotor = new WPI_TalonSRX(getPort("LiftTalonSRX", 7));
		configSRX(liftMotor);
		liftEncPort1 = new DigitalInput(getPort("1LiftEnc", 4));
		liftEncPort2 = new DigitalInput(getPort("2LiftEnc", 5));
		liftEnc = new Encoder(liftEncPort1, liftEncPort2);
		liftEnc.setPIDSourceType(PIDSourceType.kDisplacement);
		climberMotor = new WPI_TalonSRX(getPort("ClimberTalonSRX", 6));
		configSRX(climberMotor);

		leftIntakeMotor = new VictorSP(getPort("IntakeLeftVictorSP", 8));
		rightIntakeMotor = new VictorSP(getPort("IntakeRightVictorSP", 9));
		leftIntakeHorizontalSolenoid = new DoubleSolenoid(getPort("IntakeLeftHorizontalSolenoidPort1", 4),
				getPort("IntakeLeftHorizontalSolenoidPort2", 5));
		rightIntakeHorizontalSolenoid = new DoubleSolenoid(getPort("IntakeRightHorizontalSolenoidPort1", 2),
				getPort("IntakeRightHorizontalSolenoidPort2", 3));
		// leftIntakeVerticalSolenoid = new
		// DoubleSolenoid(getPort("IntakeLeftVerticalSolenoidPort1", 6),
		// getPort("IntakeLeftVerticalSolenoidPort2", 7));
		// rightIntakeVerticalSolenoid = new
		// DoubleSolenoid(getPort("IntakeRightVerticalSolenoidPort1", 8),
		// getPort("IntakeRightVerticalSolenoidPort2", 9));

		leftEncPort1 = new DigitalInput(getPort("1LeftEnc", 2));
		leftEncPort2 = new DigitalInput(getPort("2LeftEnc", 3));
		leftEncDist = new Encoder(leftEncPort1, leftEncPort2);
		leftEncDist.setPIDSourceType(PIDSourceType.kDisplacement);
		leftEncRate = new Encoder(leftEncPort1, leftEncPort2);
		leftEncRate.setPIDSourceType(PIDSourceType.kRate);
		leftEncDist.setDistancePerPulse(Robot.getConst("DPP", DIST_PER_PULSE_RATIO));
		leftEncRate.setDistancePerPulse(Robot.getConst("DPP", DIST_PER_PULSE_RATIO));

		dtLeftMaster = new WPI_TalonSRX(getPort("LeftTalonSRXMaster", 1));
		configSRX(dtLeftMaster);
		dtLeftSlave = new WPI_VictorSPX(getPort("LeftVictorSPXSlave", 2));
		configSPX(dtLeftSlave);
		dtLeft = new SpeedControllerGroup(dtLeftMaster, dtLeftSlave);
		// inverted bc gear boxes invert from input to output
		dtLeft.setInverted(true);

		rightEncPort1 = new DigitalInput(getPort("1RightEnc", 1));
		rightEncPort2 = new DigitalInput(getPort("2RightEnc", 0));
		rightEncDist = new Encoder(rightEncPort1, rightEncPort2);
		rightEncDist.setPIDSourceType(PIDSourceType.kDisplacement);
		rightEncRate = new Encoder(rightEncPort1, rightEncPort2);
		rightEncRate.setPIDSourceType(PIDSourceType.kRate);
		rightEncDist.setDistancePerPulse(Robot.getConst("DPP", DIST_PER_PULSE_RATIO));
		rightEncRate.setDistancePerPulse(Robot.getConst("DPP", DIST_PER_PULSE_RATIO));
		rightEncRate.setReverseDirection(true);

		dtRightMaster = new WPI_TalonSRX(getPort("RightTalonSRXMaster", 4));
		configSRX(dtRightMaster);
		dtRightSlave = new WPI_VictorSPX(getPort("RightVictorSPXSlave", 3));
		configSPX(dtRightSlave);
		dtRight = new SpeedControllerGroup(dtRightMaster, dtRightSlave);
		// inverted bc gear boxes invert from input to output
		dtRight.setInverted(true);

		distEncAvg = new PIDSourceAverage(leftEncDist, rightEncDist);
		fancyGyro = new AHRS(SPI.Port.kMXP);
		dtGear = new DoubleSolenoid(getPort("1dtGearSolenoid", 0), getPort("2dtGearSolenoid", 1));

		calcDefkD(Robot.getConst("Max Low Speed", 84));
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
			if (!SmartDashboard.putNumber("Port/" + key, def)) {
				System.err.println("SmartDashboard Key" + "Port/" + key + "already taken by a different type");
				return def;
			}
		}
		return (int) SmartDashboard.getNumber("Port/" + key, def);
	}

	/**
	 * Uses SmartDashboard and math to calculate a *great* default kD
	 */
	public double calcDefkD(double maxSpeed) {
		/*
		 * timeConstant is proportional to max speed of the shaft (which is the max
		 * speed of the cim divided by the gear reduction), half the mass (because the
		 * half of the drivetrain only has to support half of the robot), and radius of
		 * the drivetrain wheels squared. It's inversely proportional to the stall
		 * torque of the shaft, which is found by multiplying the stall torque of the
		 * motor with the gear reduction by the amount of motors. The omegaMax needs to
		 * be converted from rpm to radians per second, so divide by 60 and multiply to
		 * get radians.
		 */
		double timeConstant = getDrivetrainTimeConstant();
		double cycleTime = getCycleTime();
		/*
		 * The denominator of kD is 1-(e ^ -cycleTime / timeConstant). The numerator is
		 * one.
		 */
		double denominator = Math.pow(Math.E, 1 * cycleTime / timeConstant) - 1;
		return 1 / denominator / maxSpeed;
	}

	/**
	 * Gets the time constant of the drivetrain, which is used to calculate PID
	 * constants.
	 * 
	 * @return time constant
	 */
	public double getDrivetrainTimeConstant() {
		double gearReduction = getGearRatio();
		double radius = getRadius();
		double timeConstant = getOmegaMax() / gearReduction / 60 * 2 * Math.PI * convertNtokG(getWeight()) / 2 * radius
				* radius / (getStallTorque() * gearReduction * 2);
		return timeConstant;
	}

	public double getGearRatio() {
		return SmartDashboard.getBoolean("High Gear", false) ? Robot.getConst("High Gear Gear Reduction", 5.392)
				: Robot.getConst("Low Gear Gear Reduction", 12.255);
	}

	public double getRadius() {
		return Robot.getConst("Radius of Drivetrain Wheel", 0.0635);
	}

	public double getOmegaMax() {
		return Robot.getConst("Omega Max", 5330);
	}

	public double getWeight() {
		return Robot.getConst("Weight of Robot", 342);
	}

	public double getCycleTime() {
		return Robot.getConst("Code cycle time", 0.05);
	}

	public double getStallTorque() {
		return Robot.getConst("Stall Torque", 2.41);
	}

	private double convertLbsTokG(double lbs) {
		// number from google ;)
		return lbs * 0.45359237;
	}

	private double convertNtokG(double newtons) {
		// weight / accel due to grav = kg
		return newtons / 9.81;
	}

	public double convertMtoIn(double meters) {
		return meters * 39.37;
	}

}
