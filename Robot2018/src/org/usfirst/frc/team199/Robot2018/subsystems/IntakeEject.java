package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.DefaultIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeEject extends Subsystem implements IntakeEjectInterface {
	private final PowerDistributionPanel pdp = RobotMap.pdp;
	private final VictorSP leftIntakeMotor = RobotMap.leftIntakeMotor;
	private final VictorSP rightIntakeMotor = RobotMap.rightIntakeMotor;
	private final DoubleSolenoid leftSolenoid = RobotMap.leftIntakeSolenoid;
	private final DoubleSolenoid rightSolenoid = RobotMap.rightIntakeSolenoid;
	private boolean leftOpen = isForw(leftSolenoid.get());
	private boolean rightOpen = isForw(rightSolenoid.get());

	/**
	 * Return whether or not the doubleSolenoid is set to open
	 * 
	 * @param val
	 *            The value of the doublesolenoid
	 */
	public boolean isForw(DoubleSolenoid.Value val) {
		return val == DoubleSolenoid.Value.kForward;
	}

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
	 * Toggles the left intake between open and closed Laura's that works because it
	 * has been TESTED
	 */
	// public void toggleLeftIntake() {
	// if (leftOpen) {
	// // set to closed
	// leftHorizontalSolenoid.set(DoubleSolenoid.Value.kReverse);
	// } else {
	// // set to open
	// leftHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
	// }
	// leftOpen = !leftOpen;
	// }

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
	 * Laura's thing that "works" because it has been tested
	 */
	// public void toggleRightIntake() {
	// if (rightOpen) {
	// // set to closed
	// rightHorizontalSolenoid.set(DoubleSolenoid.Value.kReverse);
	// } else {
	// // set to open
	// rightHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
	// }
	// rightOpen = !rightOpen;
	// }

	/**
	 * Toggles the left intake between open and closed
	 */
	public void toggleLeftIntake() {
		leftSolenoid.set(toggleHorizontal(true));
		leftOpen = !leftOpen;
		Preferences.getInstance().putBoolean("Left Horizontal Solenoid Open", leftOpen);
	}

	/**
	 * Toggles the right intake between open and closed
	 */
	public void toggleRightIntake() {
		rightSolenoid.set(toggleHorizontal(false));
		rightOpen = !rightOpen;
		Preferences.getInstance().putBoolean("Right Horizontal Solenoid Open", rightOpen);
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
