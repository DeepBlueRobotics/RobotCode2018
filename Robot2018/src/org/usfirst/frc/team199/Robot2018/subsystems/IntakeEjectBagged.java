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
 * kForward = intake closed, kReverse = intake open
 */
public class IntakeEjectBagged extends Subsystem implements IntakeEjectInterface {
	private final PowerDistributionPanel pdp = RobotMap.pdp;
	private final VictorSP leftIntakeMotor = RobotMap.leftIntakeMotor;
	private final VictorSP rightIntakeMotor = RobotMap.rightIntakeMotor;
	private final DoubleSolenoid leftSolenoid = RobotMap.leftIntakeSolenoid;
	private final DoubleSolenoid rightSolenoid = RobotMap.rightIntakeSolenoid;
	private boolean leftOpen = isOpen(leftSolenoid.get());
	private boolean rightOpen = isOpen(rightSolenoid.get());

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
		setDefaultCommand(new DefaultIntake(this));
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
	@Override
	public boolean hasCube() {
		return pdp.getCurrent(Robot.rmap.getPort("PDP Intake Left Channel", 4)) > Robot.getConst("Max Current", 39)
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
	 * Sets the left roller to run at the specified speed. For testing purposes only
	 * (so no speed multiplier, just use joystick input)
	 * 
	 * @param speed
	 *            Speed the left motor should run at
	 */
	@Override
	public void runLeftIntake(double speed) {
		leftIntakeMotor.set(speed);
	}

	/**
	 * Sets the left roller to run at the specified speed. For testing purposes only
	 * (so no speed multiplier, just use joystick input)
	 * 
	 * @param speed
	 *            Speed the left motor should run at
	 */
	@Override
	public void runRightIntake(double speed) {
		rightIntakeMotor.set(speed);
	}

	/**
	 * Spins the rollers. If motors not inverted, negative -> intaking, positive ->
	 * ejecting
	 * 
	 * @param intaking
	 *            - true if intaking, false if ejecting
	 */
	@Override
	public void runIntake(boolean intaking) {
		int intakeSign;

		if (intaking) {
			intakeSign = -1;
		} else {
			intakeSign = 1;
		}

		if (Robot.getBool("Intake Motors Inverted", false)) {
			intakeSign *= -1;
		}

		double actualSpeed = intakeSign * Robot.getConst("Intake Motor Speed Multiplier", 1);
		runLeftIntake(actualSpeed);
		runRightIntake(actualSpeed);
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
		SmartDashboard.putBoolean("Intake Open", false);
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
		SmartDashboard.putBoolean("Intake Open", true);
	}
}