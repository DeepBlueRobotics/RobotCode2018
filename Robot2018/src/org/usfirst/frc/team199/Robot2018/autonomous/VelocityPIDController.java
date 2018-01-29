package org.usfirst.frc.team199.Robot2018.autonomous;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class VelocityPIDController extends PIDController implements SpeedController {

	private SpeedControllerGroup out;

	/**
	 * Constructs a VelocityPIDContoller and invokes the super constructor
	 * (PIDController), setting the three PID constants and the source and output
	 * for this VelocityPIDController.
	 * 
	 * @param kp
	 *            the proportional PID constant
	 * @param ki
	 *            the integral PID constant
	 * @param kd
	 *            the derivative PID constant
	 * @param source
	 *            the sensor (e.g. velocity encoder) you are reading from
	 * @param output
	 *            the SpeedController you are telling what to do
	 */
	public VelocityPIDController(double kp, double ki, double kd, PIDSource source, SpeedControllerGroup output) {
		super(kp, ki, kd, source, output);
		out = output;
	}

	/**
	 * Sets the target speed
	 * 
	 * @param speed
	 *            the target speed [-1, 1]
	 */
	@Override
	public void pidWrite(double output) {
		setSetpoint(output);
	}

	/**
	 * Sets the target speed
	 * 
	 * @param speed
	 *            the target speed [-1, 1]
	 */
	@Override
	public void set(double speed) {
		setSetpoint(speed);
	}

	/**
	 * Gets the current set voltage (setpoint) sent to the output
	 * SpeedControllerGroup
	 * 
	 * @return the current set voltage (setpoint/target/goal) sent to the output
	 *         SpeedControllerGroup
	 */
	@Override
	public double get() {
		return getSetpoint();
	}

	/**
	 * 
	 * */
	@Override
	public void setInverted(boolean isInverted) {
		out.setInverted(isInverted);
	}

	/**
	 * 
	 * */
	@Override
	public boolean getInverted() {
		return out.getInverted();
	}

	/**
	 * 
	 * */
	@Override
	public void stopMotor() {
		out.stopMotor();
	}
}
