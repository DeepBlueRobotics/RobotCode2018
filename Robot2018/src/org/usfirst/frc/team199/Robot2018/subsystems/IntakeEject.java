package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.DefaultIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class IntakeEject extends Subsystem implements IntakeEjectInterface {
	private final PowerDistributionPanel pdp = RobotMap.pdp;
	private final VictorSP leftIntakeMotor = RobotMap.leftIntakeMotor;
	private final VictorSP rightIntakeMotor = RobotMap.rightIntakeMotor;
	private final DoubleSolenoid leftVerticalSolenoid = RobotMap.leftIntakeVerticalSolenoid;
	private final DoubleSolenoid rightVerticalSolenoid = RobotMap.rightIntakeVerticalSolenoid;
	private final DoubleSolenoid leftHorizontalSolenoid = RobotMap.leftIntakeHorizontalSolenoid;
	private final DoubleSolenoid rightHorizontalSolenoid = RobotMap.rightIntakeHorizontalSolenoid;
	private boolean rightOpen = false;
	private boolean leftOpen = false;

	/**
	 * Set the default command for a subsystem here.
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new DefaultIntake());
	}

	/**
	 * @return current left motor value
	 */
	public double getLeftIntakeSpeed() {
		return leftIntakeMotor.get();
	}

	/**
	 * @return current right motor value
	 */
	public double getRightIntakeSpeed() {
		return rightIntakeMotor.get();
	}

	/**
	 * Uses current to check if the wheels are blocked aka the cube is inside the
	 * robot
	 * 
	 */
	public boolean hasCube() {
		return pdp.getCurrent(Robot.rmap.getPort("PDP Intake Left Channel", 4)) > Robot.getConst("Max Current", 39)
				|| pdp.getCurrent(Robot.rmap.getPort("PDP Intake Right Channel", 11)) > Robot.getConst("Max Current",
						39);
	}

	/**
	 * stops the motors
	 * 
	 */
	public void stopIntake() {
		leftIntakeMotor.stopMotor();
		rightIntakeMotor.stopMotor();
	}

	/**
	 * Sets the left roller to run at the specified speed
	 * 
	 * @param speed
	 *            Speed the left motor should run at
	 */
	public void runLeftIntake(double speed) {
		double actualSpeed = speed * Robot.getConst("Intake Motor Speed Multiplier", 0.5);
		leftIntakeMotor.set(actualSpeed);
	}

	/**
	 * Sets the left roller to run at the specified speed
	 * 
	 * @param speed
	 *            Speed the left motor should run at
	 */
	public void runRightIntake(double speed) {
		double actualSpeed = speed * Robot.getConst("Intake Motor Speed Multiplier", 0.5);
		rightIntakeMotor.set(actualSpeed);
	}

	/**
	 * Spins the rollers
	 * 
	 * @param speed
	 *            - negative -> rollers in, positive -> rollers out
	 */
	public void runIntake(double speed) {
		runLeftIntake(speed);
		runRightIntake(speed);
	}

	/**
	 * Raises the intake
	 */
	public void raiseIntake() {
		DoubleSolenoid.Value leftSet = Robot.getBool("Intake Left Vertical Solenoid Inverted", false)
				? DoubleSolenoid.Value.kReverse
				: DoubleSolenoid.Value.kForward;
		DoubleSolenoid.Value rightSet = Robot.getBool("Intake Right Vertical Solenoid Inverted", false)
				? DoubleSolenoid.Value.kReverse
				: DoubleSolenoid.Value.kForward;
		leftVerticalSolenoid.set(leftSet);
		rightVerticalSolenoid.set(rightSet);
	}

	/**
	 * Lowers the intake
	 */
	public void lowerIntake() {
		DoubleSolenoid.Value leftSet = Robot.getBool("Intake Left Vertical Solenoid Inverted", false)
				? DoubleSolenoid.Value.kForward
				: DoubleSolenoid.Value.kReverse;
		DoubleSolenoid.Value rightSet = Robot.getBool("Intake Right Vertical Solenoid Inverted", false)
				? DoubleSolenoid.Value.kForward
				: DoubleSolenoid.Value.kReverse;
		leftVerticalSolenoid.set(leftSet);
		rightVerticalSolenoid.set(rightSet);
	}

	/**
	 * Toggles the left intake between open and closed
	 */
	public void toggleLeftIntake() {
		// DoubleSolenoid.Value set;
		// if (Robot.getBool("Left Horizontal Solenoid Open", true)) {
		// set = Robot.getBool("Intake Left Horizontal Solenoid Inverted", false) ?
		// DoubleSolenoid.Value.kForward
		// : DoubleSolenoid.Value.kReverse;
		// } else {
		// set = Robot.getBool("Intake Left Horizontal Solenoid Inverted", false) ?
		// DoubleSolenoid.Value.kReverse
		// : DoubleSolenoid.Value.kForward;
		// }
		// leftHorizontalSolenoid.set(set);
		// SmartDashboard.putBoolean("Bool/Left Horizontal Solenoid Open",
		// !Robot.getBool("Left Horizontal Solenoid Open", true));
		if (leftOpen) {
			// set to closed
			leftHorizontalSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			// set to open
			leftHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		leftOpen = !leftOpen;
	}

	/**
	 * Toggles the right intake between open and closed
	 */
	public void toggleRightIntake() {
		// DoubleSolenoid.Value set;
		// if (Robot.getBool("Right Horizontal Solenoid Open", true)) {
		// set = Robot.getBool("Intake Right Horizontal Solenoid Inverted", false) ?
		// DoubleSolenoid.Value.kForward
		// : DoubleSolenoid.Value.kReverse;
		// } else {
		// set = Robot.getBool("Intake Right Horizontal Solenoid Inverted", false) ?
		// DoubleSolenoid.Value.kReverse
		// : DoubleSolenoid.Value.kForward;
		// }
		// rightHorizontalSolenoid.set(set);
		// SmartDashboard.putBoolean("Bool/Right Horizontal Solenoid Open",
		// !Robot.getBool("Right Horizontal Solenoid Open", true));
		if (rightOpen) {
			// set to closed
			rightHorizontalSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			// set to open
			rightHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		rightOpen = !rightOpen;
	}

	/**
	 * Closes the intake
	 */
	public void closeIntake() {
		DoubleSolenoid.Value leftSet = Robot.getBool("Intake Left Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kReverse
				: DoubleSolenoid.Value.kForward;
		DoubleSolenoid.Value rightSet = Robot.getBool("Intake Right Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kReverse
				: DoubleSolenoid.Value.kForward;
		SmartDashboard.putBoolean("Bool/Left Horizontal Solenoid Open", false);
		SmartDashboard.putBoolean("Bool/Right Horizontal Solenoid Open", false);
		leftHorizontalSolenoid.set(leftSet);
		rightHorizontalSolenoid.set(rightSet);
		leftOpen = false;
		rightOpen = false;
	}

	/**
	 * Opens the intake
	 */
	public void openIntake() {
		DoubleSolenoid.Value leftSet = Robot.getBool("Intake Left Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kForward
				: DoubleSolenoid.Value.kReverse;
		DoubleSolenoid.Value rightSet = Robot.getBool("Intake Right Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kForward
				: DoubleSolenoid.Value.kReverse;
		SmartDashboard.putBoolean("Bool/Left Horizontal Solenoid Open", true);
		SmartDashboard.putBoolean("Bool/Right Horizontal Solenoid Open", true);
		leftHorizontalSolenoid.set(leftSet);
		rightHorizontalSolenoid.set(rightSet);
		leftOpen = false;
		rightOpen = false;
	}
}
