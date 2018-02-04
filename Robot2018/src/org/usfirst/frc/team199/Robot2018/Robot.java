
package org.usfirst.frc.team199.Robot2018;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Position;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Strategy;
import org.usfirst.frc.team199.Robot2018.subsystems.Climber;
import org.usfirst.frc.team199.Robot2018.subsystems.ClimberAssist;
import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEject;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
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
public class Robot extends TimedRobot {

	public static Climber climber;
	public static ClimberAssist climberAssist;
	public static IntakeEject intakeEject;
	public static Lift lift;
	public static RobotMap rmap;
	public static Drivetrain dt;
	public static Listener listen;

	public static OI oi;
	
	public static Map<String, ArrayList<String[]>> autoScripts;

	Command autonomousCommand;
	SendableChooser<Position> posChooser = new SendableChooser<Position>();
	Map<String, SendableChooser<Strategy>> stratChoosers = new HashMap<String, SendableChooser<Strategy>>();
	String[] fmsPossibilities = {"LL", "LR", "RL", "RR"};

	public static double getConst(String key, double def) {
		if (!SmartDashboard.containsKey("Const/" + key)) {
			SmartDashboard.putNumber("Const/" + key, def);
		}
		return SmartDashboard.getNumber("Const/" + key, def);
	}

	public static boolean getBool(String key, boolean def) {
		if (!SmartDashboard.containsKey("Bool/" + key)) {
			SmartDashboard.putBoolean("Bool/" + key, def);
		}
		return SmartDashboard.getBoolean("Bool/" + key, def);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		rmap = new RobotMap();
		climber = new Climber();
		climberAssist = new ClimberAssist();
		intakeEject = new IntakeEject();
		lift = new Lift();
		dt = new Drivetrain();
		oi = new OI();
		
		// auto position chooser
		for (Position p : Position.values()) {
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
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called once during the start of autonomous in order to 
	 * grab values from SmartDashboard and the FMS and call the Autonomous
	 * command with those values.
	 */
	@Override
	public void autonomousInit() {
		String fmsInput = DriverStation.getInstance().getGameSpecificMessage();
		Position startPos = posChooser.getSelected();
		double autoDelay = SmartDashboard.getNumber("Auto Delay", 0);
		
		Map<String, Strategy> strategies = new HashMap<String, Strategy>();
		for (Map.Entry<String, SendableChooser<Strategy>> entry : stratChoosers.entrySet()) {
		    String key = entry.getKey();
		    SendableChooser<Strategy> chooser = entry.getValue();
		    strategies.put(key, chooser.getSelected());
		}
		
		Autonomous auto = new Autonomous(startPos, strategies, autoDelay, fmsInput, false);
		auto.start();
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
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	boolean firstTime = true;
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
//		if(firstTime) {
//			Robot.dt.enableVelocityPIDs();
//			firstTime = false;
//		}
		Robot.dt.setVPIDs(Robot.getConst("VPID Test Set", 0.5));
		SmartDashboard.putNumber("Drivetrain/Left VPID Targ", Robot.dt.getLeftVPIDSetpoint());
		SmartDashboard.putNumber("Drivetrain/Right VPID Targ", Robot.dt.getRightVPIDSetpoint());
		SmartDashboard.putNumber("Left VPID Error", Robot.dt.getLeftVPIDerror());
		SmartDashboard.putNumber("Right VPID Error", Robot.dt.getRightVPIDerror());
		SmartDashboard.putNumber("Left Enc Rate", Robot.dt.getLeftEncRate());
		SmartDashboard.putNumber("Right Enc Rate", Robot.dt.getRightEncRate());
		
	}
}
