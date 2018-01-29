/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018;

import org.usfirst.frc.team199.Robot2018.commands.PIDMove;
import org.usfirst.frc.team199.Robot2018.commands.PIDTurn;
import org.usfirst.frc.team199.Robot2018.commands.SetDistancePerPulse;
import org.usfirst.frc.team199.Robot2018.commands.ShiftDriveType;
import org.usfirst.frc.team199.Robot2018.commands.ShiftHighGear;
import org.usfirst.frc.team199.Robot2018.commands.ShiftLowGear;
import org.usfirst.frc.team199.Robot2018.commands.UpdatePIDConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public Joystick leftJoy;
	private JoystickButton shiftLowGear;
	private JoystickButton shiftHighGear;
	private JoystickButton shiftDriveType;
	private JoystickButton PIDMove;
	private JoystickButton PIDTurn;
	public Joystick rightJoy;
	private JoystickButton updatePidConstants;
	private JoystickButton updateEncoderDPP;
	public Joystick manipulator;

	public int getButton(String key, int def) {
		if (!SmartDashboard.containsKey("Button/" + key)) {
			SmartDashboard.putNumber("Button/" + key, def);
		}
		return (int) SmartDashboard.getNumber("Button/" + key, def);
	}

	public OI() {
		leftJoy = new Joystick(0);
		shiftLowGear = new JoystickButton(leftJoy, getButton("Shift Low Gear", 3));
		shiftLowGear.whenPressed(new ShiftLowGear());
		shiftHighGear = new JoystickButton(leftJoy, getButton("Shift High Gear", 5));
		shiftHighGear.whenPressed(new ShiftHighGear());
		shiftDriveType = new JoystickButton(leftJoy, getButton("Shift Drive Type", 2));
		shiftDriveType.whenPressed(new ShiftDriveType());
		PIDMove = new JoystickButton(leftJoy, getButton("PID Move", 7));
		PIDMove.whenPressed(new PIDMove(40, Robot.dt, RobotMap.distEncAvg));
		PIDTurn = new JoystickButton(leftJoy, getButton("PID Turn", 8));
		PIDTurn.whenPressed(new PIDTurn(30, Robot.dt, RobotMap.fancyGyro));

		rightJoy = new Joystick(1);
		updatePidConstants = new JoystickButton(rightJoy, getButton("Get PID Constants", 8));
		updatePidConstants.whenPressed(new UpdatePIDConstants());
		updateEncoderDPP = new JoystickButton(rightJoy, getButton("Get Encoder Dist Per Pulse", 9));
		updateEncoderDPP.whenPressed(new SetDistancePerPulse());

		manipulator = new Joystick(2);
	}
}
