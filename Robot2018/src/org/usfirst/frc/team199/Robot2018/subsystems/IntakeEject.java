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

	private boolean leftOpen = false;
	private boolean rightOpen = leftOpen;

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
		double actualSpeed = speed * Robot.getConst("Intake Motor Left Speed Multiplier", 1);
		leftIntakeMotor.set(actualSpeed);
	}

	/**
	 * Sets the left roller to run at the specified speed
	 * 
	 * @param speed
	 *            Speed the left motor should run at
	 */
	public void runRightIntake(double speed) {
		double actualSpeed = speed * Robot.getConst("Intake Motor Right Speed Multiplier", 1);
		rightIntakeMotor.set(actualSpeed);
	}

	/**
	 * Spins the rollers
	 * 
	 * @param speed
	 *            - positive -> rollers in, negative -> rollers out
	 */
	public void runIntake(double speed) {
		runLeftIntake(speed);
		runRightIntake(speed);
	}

	/**
	 * Raises the intake
	 */
	public void raiseIntake() {
		leftVerticalSolenoid.set(solenoidSet(true, true));
		rightVerticalSolenoid.set(solenoidSet(false, true));
	}

	/**
	 * Lowers the intake
	 */
	public void lowerIntake() {
		leftVerticalSolenoid.set(solenoidSet(true, false));
		rightVerticalSolenoid.set(solenoidSet(false, false));
	}

	/**
	 * Takes into account SmartDashboard keys to set the position of one of the
	 * vertical solenoids
	 * 
	 * @param left
	 *            Whether or not the solenoid to toggle is the left solenoid
	 * @param up
	 *            Whether or not the solenoid should be set to raised
	 * @return The DoubleSolenoid.Value that the solenoid should be set to
	 */
	public DoubleSolenoid.Value solenoidSet(boolean left, boolean up) {
		String side = left ? "Left" : "Right";
		String key = "Intake " + side + " Vertical Solenoid Inverted";
		boolean inverted = Robot.getBool(key, false);
		if ((up && inverted) || (!up && !inverted)) {
			return DoubleSolenoid.Value.kReverse;
		} else {
			return DoubleSolenoid.Value.kForward;
		}
	}

	/**
	 * Takes into account SmartDashboard keys and current position to toggle the
	 * position of one of the horizontal solenoids
	 * 
	 * @param left
	 *            Whether or not the solenoid to toggle is the left solenoid
	 * @return The DoubleSolenoid.Value that the solenoid should be set to
	 */
	public DoubleSolenoid.Value toggleHorizontal(boolean left) {
		boolean open = left ? leftOpen : rightOpen;
		String side = left ? "Left" : "Right";
		String key = "Intake " + side + " Horizontal Solenoid Inverted";
		boolean inverted = Robot.getBool(key, false);
		if ((open && inverted) || (!open && !inverted)) {
			return DoubleSolenoid.Value.kForward;
		} else {
			return DoubleSolenoid.Value.kReverse;
		}
	}

	/**
	 * Toggles the left intake between open and closed
	 */
	public void toggleLeftIntake() {
		leftHorizontalSolenoid.set(toggleHorizontal(true));
		leftOpen = !leftOpen;
		SmartDashboard.putBoolean("Left Horizontal Solenoid Open", leftOpen);
	}

	/**
	 * Toggles the right intake between open and closed
	 */
	public void toggleRightIntake() {
		rightHorizontalSolenoid.set(toggleHorizontal(false));
		rightOpen = !rightOpen;
		SmartDashboard.putBoolean("Right Horizontal Solenoid Open", rightOpen);
	}

	/**
	 * Closes the intake
	 */
	public void closeIntake() {
		if (leftOpen) {
			toggleLeftIntake();
		}
		if (rightOpen) {
			toggleRightIntake();
		}
	}

	/**
	 * Opens the intake
	 */
	public void openIntake() {
		if (!leftOpen) {
			toggleLeftIntake();
		}
		if (!rightOpen) {
			toggleRightIntake();
		}
	}
}
