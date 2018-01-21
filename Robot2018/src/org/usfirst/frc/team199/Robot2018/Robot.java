
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
	
	public static final Climber climber = new Climber();
	public static final ClimberAssist climberAssist = new ClimberAssist();
	public static final IntakeEject intakeEject = new IntakeEject();
	public static final Lift lift = new Lift();
	public static OI oi;
	
	public static Map<String, ArrayList<String[]>> autoScripts;

	Command autonomousCommand;
	SendableChooser<Position> posChooser = new SendableChooser<Position>();
	Map<String, SendableChooser<Strategy>> stratChoosers = new HashMap<String, SendableChooser<Strategy>>();
	String[] fmsPossibilities = {"LL", "LR", "RL", "RR"};

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		
		// put in position chooer
		for (Position p : Position.values()) {
			posChooser.addObject(p.getSDName(), p);
		}
		SmartDashboard.putData("Starting Position", posChooser);
		
		// put in strategy choosers
		for (String input : fmsPossibilities) {
			SendableChooser<Strategy> chooser = new SendableChooser<Strategy>();
			for (Strategy s : Strategy.values()) {
				chooser.addObject(s.getSDName(), s);
			}
			SmartDashboard.putData(input, chooser);
			stratChoosers.put(input, chooser);
		}
		
		SmartDashboard.putNumber("Auto Delay", 0);

		autoScripts = AutoUtils.parseScriptFile(Preferences.getInstance().getString("autoscripts", ""));
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings &amp; commands.
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
		
		Autonomous auto = new Autonomous(startPos, strategies, autoDelay, fmsInput);
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

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}
