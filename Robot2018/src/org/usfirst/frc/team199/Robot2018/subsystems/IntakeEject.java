package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * kForward = intake closed, kReverse = intake open
 */
public class IntakeEject extends Subsystem implements IntakeEjectInterface {
	private final PowerDistributionPanel pdp = RobotMap.pdp;
	private final VictorSP leftIntakeMotor = RobotMap.leftIntakeMotor;
	private final VictorSP rightIntakeMotor = RobotMap.rightIntakeMotor;
	private final DoubleSolenoid leftSolenoid = RobotMap.leftIntakeSolenoid;
	private final DoubleSolenoid rightSolenoid = RobotMap.rightIntakeSolenoid;
	private boolean leftOpen = isOpen(leftSolenoid.get());
	private boolean rightOpen = isOpen(rightSolenoid.get());
	private boolean hasCube = false;

	/**
	 * Return whether or not a side of the intake (L/R) is open
	 * 
	 * @param val
	 *            The value (kForward, kReverse, kOff) of the DoubleSolenoid for one
	 *            side of the intake
	 */
	public boolean isOpen(DoubleSolenoid.Value val) {
		return val == DoubleSolenoid.Value.kReverse;
	}

	/**
	 * Set the default command for a subsystem here.
	 */
	@Override
	public void initDefaultCommand() {
		// I don't want this on the manipulator joysticks during a match
		// setDefaultCommand(new DefaultIntake());
	}

	/**
	 * @return current left motor value
	 */
	@Override
	public double getLeftIntakeSpeed() {
		return leftIntakeMotor.get();
	}

	/**
	 * @return current right motor value
	 */
	@Override
	public double getRightIntakeSpeed() {
		return rightIntakeMotor.get();
	}

	/**
	 * Uses current to check if the wheels are blocked aka the cube is inside the
	 * robot
	 * 
	 */
	@Override
	public boolean hasCube() {
		SmartDashboard.putNumber("Intake Current Left", Robot.rmap.getPort("PDP Intake Left Channel", 4));

		SmartDashboard.putNumber("Intake Current Right", Robot.rmap.getPort("PDP Intake Left Channel", 11));
		return pdp.getCurrent(Robot.rmap.getPort("PDP Intake Left Channel", 4)) > Robot.getConst("Max Current", 15)
				|| pdp.getCurrent(Robot.rmap.getPort("PDP Intake Right Channel", 11)) > Robot.getConst("Max Current",
						39);
	}

	/**
	 * stops the motors
	 * 
	 */
	@Override
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
	@Override
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
	@Override
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
	@Override
	public void runIntake(double speed) {
		runLeftIntake(speed);
		runRightIntake(speed);
	}

	/**
	 * Toggles the left intake between open (kReverse) and closed (kForward).
	 */
	@Override
	public void toggleLeftIntake() {
		if (leftOpen) {
			// set to closed
			leftSolenoid.set(DoubleSolenoid.Value.kForward);
		} else {
			// set to open
			leftSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
		leftOpen = !leftOpen;
		SmartDashboard.putBoolean("Left Solenoid Open", leftOpen);
	}

	/**
	 * Toggles the right intake between open (kReverse) and closed (kForward).
	 */
	@Override
	public void toggleRightIntake() {
		if (rightOpen) {
			// set to closed
			rightSolenoid.set(DoubleSolenoid.Value.kForward);
		} else {
			// set to open
			rightSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
		rightOpen = !rightOpen;
		SmartDashboard.putBoolean("Right Solenoid Open", rightOpen);
	}

	/**
	 * Closes the intake
	 */
	@Override
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
	@Override
	public void openIntake() {
		if (!leftOpen) {
			toggleLeftIntake();
		}
		if (!rightOpen) {
			toggleRightIntake();
		}
	}
}
