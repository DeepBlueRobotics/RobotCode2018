/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
import org.usfirst.frc.team199.Robot2018.autonomous.State;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Strategy;
import org.usfirst.frc.team199.Robot2018.commands.CloseIntake;
import org.usfirst.frc.team199.Robot2018.commands.ShiftLowGear;
import org.usfirst.frc.team199.Robot2018.subsystems.Climber;
import org.usfirst.frc.team199.Robot2018.subsystems.ClimberAssist;
import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEject;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static Climber climber;
	public static ClimberAssist climberAssist;
	public static IntakeEject intakeEject;
	public static Lift lift;
	public static RobotMap rmap;
	public static Drivetrain dt;
	public static Listener listen;
	public static Robot robot;

	public static OI oi;

	public static Map<String, ArrayList<String[]>> autoScripts;

	public static boolean stopIntake = false;

	Command autonomousCommand;
	SendableChooser<Autonomous.Position> posChooser = new SendableChooser<Autonomous.Position>();
	Map<String, SendableChooser<Strategy>> stratChoosers = new HashMap<String, SendableChooser<Strategy>>();
	String[] fmsPossibilities = { "LL", "LR", "RL", "RR" };

	public static SmartDashboardInterface sd = new SmartDashboardInterface() {
		@Override
		public double getConst(String key, double def) {
			Preferences pref = Preferences.getInstance();
			if (!pref.containsKey("Const/" + key)) {
				pref.putDouble("Const/" + key, def);
				if (pref.getDouble("Const/ + key", def) != def) {
					System.err.println("pref Key" + "Const/" + key + "already taken by a different type");
					return def;
				}
			}
			return pref.getDouble("Const/" + key, def);
		}

		@Override
		public String getString(String key, String def) {
			Preferences pref = Preferences.getInstance();
			if (!pref.containsKey("String/" + key)) {
				pref.putString("String/" + key, def);
				if (pref.getString("String/ + key", def) != def) {
					System.err.println("pref Key" + "String/" + key + "already taken by a different type");
					return def;
				}
			}
			return pref.getString("String/" + key, def);
		}

		@Override
		public void putConst(String key, double def) {
			Preferences pref = Preferences.getInstance();
			pref.putDouble("Const/" + key, def);
			if (pref.getDouble("Const/ + key", def) != def) {
				System.err.println("pref Key" + "Const/" + key + "already taken by a different type");
			}
		}

		@Override
		public void putData(String string, PIDController controller) {
			SmartDashboard.putData(string, controller);
		}

		@Override
		public void putNumber(String string, double d) {
			SmartDashboard.putNumber(string, d);
		}

		@Override
		public void putBoolean(String string, boolean b) {
			SmartDashboard.putBoolean(string, b);
		}

		/*
		 * if (!SmartDashboard.containsKey("Const/" + key)) { if
		 * (!SmartDashboard.putNumber("Const/" + key, def)) {
		 * System.err.println("SmartDashboard Key" + "Const/" + key +
		 * "already taken by a different type"); return def; } } return
		 * SmartDashboard.getNumber("Const/" + key, def);
		 */
	};

	public static double getConst(String key, double def) {
		return sd.getConst(key, def);
	}

	public static String getString(String key, String def) {
		return sd.getString(key, def);
	}

	public static void putConst(String key, double def) {
		sd.putConst(key, def);
	}

	public static boolean getBool(String key, boolean def) {
		Preferences pref = Preferences.getInstance();
		if (!pref.containsKey("Bool/" + key)) {
			pref.putBoolean("Bool/" + key, def);
			if (pref.getBoolean("Bool/" + key, def) == def) {
				System.err.println("pref Key" + "Bool/" + key + "already taken by a different type");
				return def;
			}
		}
		return pref.getBoolean("Bool/" + key, def);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		robot = this;
		rmap = new RobotMap();
		climber = new Climber();
		climberAssist = new ClimberAssist();
		intakeEject = new IntakeEject();
		lift = new Lift();
		dt = new Drivetrain(sd);
		oi = new OI(this);

		// auto position chooser
		for (Autonomous.Position p : Autonomous.Position.values()) {
			posChooser.addObject(p.getSDName(), p);
		}
		SmartDashboard.putData("Starting Position", posChooser);

		// auto strategy choosers
		for (String input : fmsPossibilities) {
			SendableChooser<Strategy> chooser = new SendableChooser<Strategy>();
			for (Strategy s : Strategy.values()) {
				chooser.addObject(s.getSDName(), s);
			}
			SmartDashboard.putData(input, chooser);
			stratChoosers.put(input, chooser);
		}

		// auto delay chooser
		SmartDashboard.putNumber("Auto Delay", 0);

		// parse scripts from Preferences, which maintains values throughout reboots
		autoScripts = AutoUtils.parseScriptFile(Preferences.getInstance().getString("autoscripts", ""));

		listen = new Listener();
		lift.resetEnc();
		// CameraServer.getInstance().startAutomaticCapture(0);
		CameraServer.getInstance().startAutomaticCapture((int) Robot.getConst("Camera Port", 1));
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		dt.disableVelocityPIDs();
		lift.setSetpoint(0);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called once during the start of autonomous in order to grab
	 * values from SmartDashboard and the FMS and call the Autonomous command with
	 * those values.
	 */
	@Override
	public void autonomousInit() {
		dt.resetAHRS();
		AutoUtils.state = new State(0, 0, 0);
		Scheduler.getInstance().add(new ShiftLowGear());
		Scheduler.getInstance().add(new CloseIntake());
		String fmsInput = DriverStation.getInstance().getGameSpecificMessage();
		Autonomous.Position startPos = posChooser.getSelected();
		double autoDelay = SmartDashboard.getNumber("Auto Delay", 0);

		Map<String, Strategy> strategies = new HashMap<String, Strategy>();
		for (Map.Entry<String, SendableChooser<Strategy>> entry : stratChoosers.entrySet()) {
			String key = entry.getKey();
			SendableChooser<Strategy> chooser = entry.getValue();
			strategies.put(key, chooser.getSelected());
		}

		Scheduler.getInstance().add(new Autonomous(startPos, strategies, autoDelay, fmsInput, false));
		// auto.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		System.out.println("In teleopInit()");
		dt.resetAHRS();
		AutoUtils.state = new State(0, 0, 0);
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		dt.putVelocityControllersToDashboard();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		// SmartDashboard.putNumber("Drivetrain/Left VPID Targ",
		// Robot.dt.getLeftVPIDSetpoint());
		// SmartDashboard.putNumber("Drivetrain/Right VPID Targ",
		// Robot.dt.getRightVPIDSetpoint());
		// SmartDashboard.putNumber("Left VPID Error", Robot.dt.getLeftVPIDerror());
		// SmartDashboard.putNumber("Right VPID Error", Robot.dt.getRightVPIDerror());
		// SmartDashboard.putNumber("Left Enc Rate", Robot.dt.getLeftEncRate());
		// SmartDashboard.putNumber("Right Enc Rate", Robot.dt.getRightEncRate());
		//
		// SmartDashboard.putNumber("Left Enc Dist", dt.getLeftDist());
		// SmartDashboard.putNumber("Right Enc Dist", dt.getRightDist());
		// SmartDashboard.putNumber("Avg Enc Dist", dt.getEncAvgDist());
		//

		// System.out.printf("Left: %1$5.2f; Right: %2$5.2f\n", RobotMap.dtLeft.get(),
		// RobotMap.dtRight.get());

		SmartDashboard.putNumber("Angle", dt.getAHRSAngle());
		SmartDashboard.putNumber("Left Current draw", rmap.pdp.getCurrent(4));
		SmartDashboard.putNumber("Right Current draw", rmap.pdp.getCurrent(11));
	}

	boolean firstTime = true;

	@Override
	public void testInit() {
		System.out.println("In testInit()");
		dt.resetAHRS();
		AutoUtils.state = new State(0, 0, 0);
		lift.disable();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		// Scheduler.getInstance().run();
		// lift.disable();
		// lift.runMotor(SmartDashboard.getNumber("Voltage to Lift", 0));
	}
}
