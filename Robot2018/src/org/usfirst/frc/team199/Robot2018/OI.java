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
import org.usfirst.frc.team199.Robot2018.commands.OuttakeCube;
import org.usfirst.frc.team199.Robot2018.commands.PIDMove;
import org.usfirst.frc.team199.Robot2018.commands.PIDTurn;
import org.usfirst.frc.team199.Robot2018.commands.RaiseIntake;
import org.usfirst.frc.team199.Robot2018.commands.ResetEncoders;
import org.usfirst.frc.team199.Robot2018.commands.RunLift;
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
	/*
	 * WHENEVER YOU ADD OR CHANGE WHAT A BUTTON OR JOYSTICK DOES, indicate in
	 * /docs/controllers.txt to keep that reference up to date.
	 */

	public Joystick leftJoy;
	private JoystickButton shiftLowGearButton;
	private JoystickButton shiftHighGearButton;
	private JoystickButton shiftDriveTypeButton;
	private JoystickButton pIDMoveButton;
	private JoystickButton pIDTurnButton;
	private JoystickButton resetEncButton;
	private JoystickButton moveLiftUpButton;
	private JoystickButton moveLiftDownButton;
	public Joystick rightJoy;
	private JoystickButton updatePIDConstantsButton;
	private JoystickButton updateEncoderDPPButton;
	public Joystick manipulator;
	private JoystickButton closeIntakeButton;
	private JoystickButton openIntakeButton;
	private JoystickButton raiseIntakeButton;
	private JoystickButton lowerIntakeButton;
	private JoystickButton intakeCubeButton;
	private JoystickButton outtakeCubeButton;

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
		shiftDriveTypeButton = new JoystickButton(leftJoy, getButton("Shift Drive Type", 2));
		shiftDriveTypeButton.whenPressed(new ShiftDriveType());
		pIDMoveButton = new JoystickButton(leftJoy, getButton("PID Move", 7));
		pIDMoveButton
				.whenPressed(new PIDMove(Robot.sd.getConst("Move Targ", 24), Robot.dt, Robot.sd, RobotMap.distEncAvg));
		pIDTurnButton = new JoystickButton(leftJoy, getButton("PID Turn", 8));
		// PIDTurnButton.whenPressed(new PIDTurn(Robot.getConst("Turn Targ", 90),
		// Robot.dt, Robot.sd RobotMap.fancyGyro));
		pIDTurnButton
				.whenReleased(new PIDTurn(Robot.getConst("Turn Targ", 90), Robot.dt, Robot.sd, RobotMap.fancyGyro));
		resetEncButton = new JoystickButton(leftJoy, getButton("Reset Dist Enc", 10));
		resetEncButton.whenPressed(new ResetEncoders());

		rightJoy = new Joystick(1);
		shiftHighGearButton = new JoystickButton(rightJoy, getButton("Shift High Gear", 3));
		shiftHighGearButton.whenPressed(new ShiftHighGear());
		shiftLowGearButton = new JoystickButton(rightJoy, getButton("Shift Low Gear", 2));
		shiftLowGearButton.whenPressed(new ShiftLowGear());
		updatePIDConstantsButton = new JoystickButton(rightJoy, getButton("Get PID Constants", 8));
		updatePIDConstantsButton.whenPressed(new UpdatePIDConstants());
		updateEncoderDPPButton = new JoystickButton(rightJoy, getButton("Get Encoder Dist Per Pulse", 9));
		updateEncoderDPPButton.whenPressed(new SetDistancePerPulse());
		moveLiftUpButton = new JoystickButton(rightJoy, getButton("Run Lift Motor Up", 10));
		moveLiftDownButton = new JoystickButton(rightJoy, getButton("Run Lift Motor Down", 11));
		moveLiftUpButton.whileHeld(new RunLift(Robot.lift, true));
		moveLiftDownButton.whileHeld(new RunLift(Robot.lift, false));

		manipulator = new Joystick(2);
		closeIntakeButton = new JoystickButton(manipulator, getButton("Close Intake Button", 1));
		closeIntakeButton.whenPressed(new CloseIntake());
		openIntakeButton = new JoystickButton(manipulator, getButton("Open Intake Button", 2));
		openIntakeButton.whenPressed(new OpenIntake());
		raiseIntakeButton = new JoystickButton(manipulator, getButton("Raise Intake Button", 3));
		raiseIntakeButton.whenPressed(new RaiseIntake());
		lowerIntakeButton = new JoystickButton(manipulator, getButton("Lower Intake Button", 4));
		lowerIntakeButton.whenPressed(new LowerIntake());
		intakeCubeButton = new JoystickButton(manipulator, getButton("Intake Button", 5));
		intakeCubeButton.whenPressed(new IntakeCube());
		outtakeCubeButton = new JoystickButton(manipulator, getButton("Outake Button", 6));
		outtakeCubeButton.whenPressed(new OuttakeCube());
	}
}
