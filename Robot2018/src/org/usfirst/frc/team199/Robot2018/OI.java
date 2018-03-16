/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team199.Robot2018;

import org.usfirst.frc.team199.Robot2018.commands.CloseIntake;
import org.usfirst.frc.team199.Robot2018.commands.FindTurnTimeConstant;
import org.usfirst.frc.team199.Robot2018.commands.IntakeCube;
import org.usfirst.frc.team199.Robot2018.commands.OpenIntake;
import org.usfirst.frc.team199.Robot2018.commands.OuttakeCube;
import org.usfirst.frc.team199.Robot2018.commands.PIDMove;
import org.usfirst.frc.team199.Robot2018.commands.PIDTurn;
import org.usfirst.frc.team199.Robot2018.commands.ResetEncoders;
import org.usfirst.frc.team199.Robot2018.commands.RunLift;
import org.usfirst.frc.team199.Robot2018.commands.SetDistancePerPulse;
import org.usfirst.frc.team199.Robot2018.commands.ShiftDriveType;
import org.usfirst.frc.team199.Robot2018.commands.ShiftHighGear;
import org.usfirst.frc.team199.Robot2018.commands.ShiftLowGear;
import org.usfirst.frc.team199.Robot2018.commands.StopIntake;
import org.usfirst.frc.team199.Robot2018.commands.ToggleLeftIntake;
import org.usfirst.frc.team199.Robot2018.commands.ToggleRightIntake;
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
	public Joystick rightJoy;
	private JoystickButton shiftLowGearButton;
	private JoystickButton shiftHighGearButton;
	private JoystickButton shiftDriveTypeButton;
	private JoystickButton pIDMoveButton;
	private JoystickButton pIDTurnButton;
	private JoystickButton resetEncButton;
	private JoystickButton moveLiftUpButton;
	private JoystickButton moveLiftDownButton;
	private JoystickButton findTurnTimeConstantButton;
	private JoystickButton updatePIDConstantsButton;
	private JoystickButton updateEncoderDPPButton;

	public Joystick manipulator;
	private JoystickButton closeIntakeButton;
	private JoystickButton openIntakeButton;
	private JoystickButton raiseIntakeButton;
	private JoystickButton lowerIntakeButton;
	private JoystickButton intakeCubeButton;
	private JoystickButton outakeCubeButton;
	private JoystickButton toggleLeftIntakeButton;
	private JoystickButton toggleRightIntakeButton;
	private JoystickButton stopIntakeButton;

	public int getButton(String key, int def) {
		if (!SmartDashboard.containsKey("Button/" + key)) {
			if (!SmartDashboard.putNumber("Button/" + key, def)) {
				System.err.println("SmartDashboard Key" + "Button/" + key + "already taken by a different type");
				return def;
			}
		}
		return (int) SmartDashboard.getNumber("Button/" + key, def);
	}

	public OI(Robot robot) {
		leftJoy = new Joystick(0);
		shiftDriveTypeButton = new JoystickButton(leftJoy, getButton("Shift Drive Type", 2));
		shiftDriveTypeButton.whenPressed(new ShiftDriveType());

		pIDMoveButton = new JoystickButton(leftJoy, getButton("PID Move", 7));
		pIDMoveButton
				.whenPressed(new PIDMove(Robot.sd.getConst("Move Targ", 24), Robot.dt, Robot.sd, RobotMap.distEncAvg));
		pIDTurnButton = new JoystickButton(leftJoy, getButton("PID Turn", 8));
		pIDTurnButton
				.whenReleased(new PIDTurn(Robot.getConst("Turn Targ", 90), Robot.dt, Robot.sd, RobotMap.fancyGyro));

		resetEncButton = new JoystickButton(leftJoy, getButton("Reset Dist Enc", 10));
		resetEncButton.whenPressed(new ResetEncoders());

		findTurnTimeConstantButton = new JoystickButton(leftJoy, getButton("Find Turn Time Constant", 11));
		// the command will only run in test mode
		findTurnTimeConstantButton
				.whenPressed(new FindTurnTimeConstant(robot, Robot.dt, Robot.rmap.fancyGyro, Robot.sd));

		rightJoy = new Joystick(1);
		shiftHighGearButton = new JoystickButton(rightJoy, getButton("Shift High Gear", 4));
		shiftHighGearButton.whenPressed(new ShiftHighGear());
		shiftLowGearButton = new JoystickButton(rightJoy, getButton("Shift Low Gear", 3));
		shiftLowGearButton.whenPressed(new ShiftLowGear());

		updatePIDConstantsButton = new JoystickButton(rightJoy, getButton("Get PID Constants", 8));
		updatePIDConstantsButton.whenPressed(new UpdatePIDConstants());
		updateEncoderDPPButton = new JoystickButton(rightJoy, getButton("Get Encoder Dist Per Pulse", 9));
		updateEncoderDPPButton.whenPressed(new SetDistancePerPulse());

		moveLiftUpButton = new JoystickButton(rightJoy, getButton("Run Lift Motor Up", 10));
		moveLiftUpButton.whileHeld(new RunLift(Robot.lift, true));
		moveLiftDownButton = new JoystickButton(rightJoy, getButton("Run Lift Motor Down", 11));
		moveLiftDownButton.whileHeld(new RunLift(Robot.lift, false));

		manipulator = new Joystick(2);
		if (manipulator.getButtonCount() == 0) {
			System.err.println(
					"ERROR: manipulator does not appear to be plugged in. Disabling intake code. Restart code with manipulator plugged in to enable intake code");
		} else {
			closeIntakeButton = new JoystickButton(manipulator, getButton("Close Intake Button", 1));
			closeIntakeButton.whenPressed(new CloseIntake());
			openIntakeButton = new JoystickButton(manipulator, getButton("Open Intake Button", 2));
			openIntakeButton.whenPressed(new OpenIntake());
			// raiseIntake = new JoystickButton(manipulator, getButton("Raise Intake
			// Button", 3));
			// raiseIntake.whenPressed(new RaiseIntake());
			// lowerIntake = new JoystickButton(manipulator, getButton("Lower Intake
			// Button", 4));
			// lowerIntake.whenPressed(new LowerIntake());
			intakeCubeButton = new JoystickButton(manipulator, getButton("Intake Button", 5));
			intakeCubeButton.whenPressed(new IntakeCube());
			outakeCubeButton = new JoystickButton(manipulator, getButton("Outake Button", 6));
			outakeCubeButton.whenPressed(new OuttakeCube());
			toggleLeftIntakeButton = new JoystickButton(manipulator, getButton("Toggle Left Intake Button", 3));
			toggleLeftIntakeButton.whenPressed(new ToggleLeftIntake());
			toggleRightIntakeButton = new JoystickButton(manipulator, getButton("Toggle Right Intake Button", 4));
			toggleRightIntakeButton.whenPressed(new ToggleRightIntake());

			stopIntakeButton = new JoystickButton(manipulator, getButton("Stop Intake Button", 7));
			stopIntakeButton.whenPressed(new StopIntake());
		}

	}

	// /**
	// * Returns the getY from leftJoy squared (preserving sign)
	// *
	// * @return The y value squared
	// */
	// public double squareLeftY() {
	// return leftJoy.getY() * leftJoy.getY() * Math.signum(leftJoy.getY());
	// }
	//
	// /**
	// * Returns the getY from rightJoy squared (preserving sign)
	// *
	// * @return The y value squared
	// */
	// public double squareRightY() {
	// return rightJoy.getY() * rightJoy.getY() * Math.signum(rightJoy.getY());
	// }
	//
	// /**
	// * Returns the getX from leftJoy squared (preserving sign)
	// *
	// * @return The x value squared
	// */
	// public double squareLeftX() {
	// return leftJoy.getX() * leftJoy.getX() * Math.signum(leftJoy.getX());
	// }
	//
	// /**
	// * Returns the getX from rightJoy squared (preserving sign)
	// *
	// * @return The x value squared
	// */
	// public double squareRightX() {
	// return rightJoy.getX() * rightJoy.getX() * Math.signum(rightJoy.getX());
	// }

	/**
	 * Used to square joystick values while keeping sign
	 * 
	 * @param joyVal
	 *            The joystick value to stick
	 * @return The squared joystick value with same sign
	 */
	public double squareValueKeepSign(double joyVal) {
		return joyVal * joyVal * Math.signum(joyVal);
	}
}
