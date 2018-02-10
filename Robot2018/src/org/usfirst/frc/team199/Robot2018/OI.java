/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018;

import org.usfirst.frc.team199.Robot2018.commands.CloseIntake;
import org.usfirst.frc.team199.Robot2018.commands.IntakeCube;
import org.usfirst.frc.team199.Robot2018.commands.LowerIntake;
import org.usfirst.frc.team199.Robot2018.commands.OpenIntake;
import org.usfirst.frc.team199.Robot2018.commands.PIDMove;
import org.usfirst.frc.team199.Robot2018.commands.PIDTurn;
import org.usfirst.frc.team199.Robot2018.commands.RaiseIntake;
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
	private JoystickButton shiftLowGearButton;
	private JoystickButton shiftHighGearButton;
	private JoystickButton shiftDriveTypeButton;
	private JoystickButton PIDMoveButton;
	private JoystickButton PIDTurnButton;
	public Joystick rightJoy;
	private JoystickButton updatePIDConstantsButton;
	private JoystickButton updateEncoderDPPButton;
	public Joystick manipulator;
	private JoystickButton closeIntake;
	private JoystickButton openIntake;
	private JoystickButton raiseIntake;
	private JoystickButton lowerIntake;
	private JoystickButton intake;
	private JoystickButton outake;

	public int getButton(String key, int def) {
		if (!SmartDashboard.containsKey("Button/" + key)) {
			if (!SmartDashboard.putNumber("Button/" + key, def)) {
				System.err.println("SmartDashboard Key" + "Button/" + key + "already taken by a different type");
				return def;
			}
		}
		return (int) SmartDashboard.getNumber("Button/" + key, def);
	}

	public OI() {
		leftJoy = new Joystick(0);
		shiftLowGearButton = new JoystickButton(leftJoy, getButton("Shift Low Gear", 3));
		shiftLowGearButton.whenPressed(new ShiftLowGear());
		shiftHighGearButton = new JoystickButton(leftJoy, getButton("Shift High Gear", 5));
		shiftHighGearButton.whenPressed(new ShiftHighGear());
		shiftDriveTypeButton = new JoystickButton(leftJoy, getButton("Shift Drive Type", 2));
		shiftDriveTypeButton.whenPressed(new ShiftDriveType());
		PIDMoveButton = new JoystickButton(leftJoy, getButton("PID Move", 7));
		PIDMoveButton.whenPressed(new PIDMove(40, Robot.dt, RobotMap.distEncAvg));
		PIDTurnButton = new JoystickButton(leftJoy, getButton("PID Turn", 8));
		PIDTurnButton.whenPressed(new PIDTurn(30, Robot.dt, RobotMap.fancyGyro));

		rightJoy = new Joystick(1);
		updatePIDConstantsButton = new JoystickButton(rightJoy, getButton("Get PID Constants", 8));
		updatePIDConstantsButton.whenPressed(new UpdatePIDConstants());
		updateEncoderDPPButton = new JoystickButton(rightJoy, getButton("Get Encoder Dist Per Pulse", 9));
		updateEncoderDPPButton.whenPressed(new SetDistancePerPulse());

		manipulator = new Joystick(2);
		closeIntake = new JoystickButton(manipulator, getButton("Close Intake Button", 1));
		closeIntake.whenPressed(new CloseIntake());
		openIntake = new JoystickButton(manipulator, getButton("Open Intake Button", 2));
		openIntake.whenPressed(new OpenIntake());
		raiseIntake = new JoystickButton(manipulator, getButton("Raise Intake Button", 3));
		raiseIntake.whenPressed(new RaiseIntake());
		lowerIntake = new JoystickButton(manipulator, getButton("Lower Intake Button", 4));
		lowerIntake.whenPressed(new LowerIntake());
		intake = new JoystickButton(manipulator, getButton("Intake Button", 5));
		intake.whileHeld(new IntakeCube(true));
		outake = new JoystickButton(manipulator, getButton("Outake Button", 6));
		outake.whileHeld(new IntakeCube(false));
	}
}
