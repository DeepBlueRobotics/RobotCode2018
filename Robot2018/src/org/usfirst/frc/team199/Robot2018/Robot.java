
package org.usfirst.frc.team199.Robot2018;

import java.util.ArrayList;

import org.usfirst.frc.team199.Robot2018.subsystems.Climber;
import org.usfirst.frc.team199.Robot2018.subsystems.ClimberAssist;
import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEject;
import org.usfirst.frc.team199.Robot2018.subsystems.Lift;

import edu.wpi.first.wpilibj.IterativeRobot;
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

	public static final Climber climber = new Climber();
	public static final ClimberAssist climberAssist = new ClimberAssist();
	public static final IntakeEject intakeEject = new IntakeEject();
	public static final Lift lift = new Lift();
	public static RobotMap rmap;
	public static Drivetrain dt;

	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	public static double getConst(String key, double def) {
		if (!SmartDashboard.containsKey(key)) {
			SmartDashboard.putNumber(key, def);
		}
		return SmartDashboard.getNumber(key, def);
	}

	public void sendValuesToDashboard() {
		ArrayList<String> boolKeys = new ArrayList<String>();
		ArrayList<Boolean> boolDef = new ArrayList<Boolean>();
		boolKeys.add("Arcade Drive");
		boolKeys.add("Arcade Drive Default Setup");
		boolKeys.add("Square Drive Values");
		boolKeys.add("High Gear");

		boolDef.add(true);
		boolDef.add(true);
		boolDef.add(false);
		boolDef.add(false);
		for (int i = 0; i < boolKeys.size(); i++) {
			if (!SmartDashboard.containsKey(boolKeys.get(i))) {
				SmartDashboard.putBoolean(boolKeys.get(i), boolDef.get(i));
			}
		}
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		rmap = new RobotMap();
		dt = new Drivetrain();
		oi = new OI();
		sendValuesToDashboard();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
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
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
		 * ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
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
