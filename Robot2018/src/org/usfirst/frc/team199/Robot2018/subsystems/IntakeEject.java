package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

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

	/**
	 * Set the default command for a subsystem here.
	 */
	public void initDefaultCommand() {
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
		return pdp.getCurrent(Robot.rmap.getPort("PDP Intake Left Channel", 4)) > Robot.getConst("Max Current", 38)
				&& pdp.getCurrent(Robot.rmap.getPort("PDP Intake Right Channel", 11)) > Robot.getConst("Max Current",
						38);
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
	 * Spins the rollers
	 * 
	 * @param speed
	 *            - positive -> rollers in, negative -> rollers out
	 */
	public void runIntake(double speed) {
		double actualSpeed = speed * Robot.getConst("Intake Motor Speed Multiplier", 1);
		leftIntakeMotor.set(actualSpeed);
		rightIntakeMotor.set(actualSpeed);
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
	 * Opens the intake
	 */
	public void openIntake() {
		DoubleSolenoid.Value leftSet = Robot.getBool("Intake Left Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kReverse
				: DoubleSolenoid.Value.kForward;
		DoubleSolenoid.Value rightSet = Robot.getBool("Intake Right Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kReverse
				: DoubleSolenoid.Value.kForward;
		leftHorizontalSolenoid.set(leftSet);
		rightHorizontalSolenoid.set(rightSet);
	}

	/**
	 * Closes the intake
	 */
	public void closeIntake() {
		DoubleSolenoid.Value leftSet = Robot.getBool("Intake Left Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kForward
				: DoubleSolenoid.Value.kReverse;
		DoubleSolenoid.Value rightSet = Robot.getBool("Intake Right Horizontal Solenoid Inverted", false)
				? DoubleSolenoid.Value.kForward
				: DoubleSolenoid.Value.kReverse;
		leftHorizontalSolenoid.set(leftSet);
		rightHorizontalSolenoid.set(rightSet);
	}
}
