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
	 * returns current left motor value
	 */
	public double getLeftIntakeSpeed() {
		return leftIntakeMotor.get();
	}

	/**
	 * returns current right motor value
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
		leftVerticalSolenoid.set(DoubleSolenoid.Value.kForward);
		rightVerticalSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	/**
	 * Lowers the intake
	 */
	public void lowerIntake() {
		leftVerticalSolenoid.set(DoubleSolenoid.Value.kReverse);
		rightVerticalSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	/**
	 * Opens the intake
	 */
	public void openIntake() {
		leftHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
		rightHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	/**
	 * Closes the intake
	 */
	public void closeIntake() {
		leftHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
		rightHorizontalSolenoid.set(DoubleSolenoid.Value.kForward);
	}
}
