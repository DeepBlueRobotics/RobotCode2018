package org.usfirst.frc.team199.Robot2018.autonomous;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class VelocityPIDController extends PIDController implements SpeedController {

	private SpeedControllerGroup out;

	/**
	 * Constructs a VelocityPIDContoller and invokes the super constructor
	 * (PIDController), setting the four PID constants and the source and output for
	 * this VelocityPIDController.
	 * 
	 * @param kp
	 *            the proportional PID constant, multiplied by the total error (aka
	 *            I)
	 * @param ki
	 *            the integral PID constant, not used in velocity control
	 * @param kd
	 *            the derivative PID constant, multiplied by the error (aka P)
	 * @param kf
	 *            the feed forward value: should be 1/MaxSpeed
	 * @param source
	 *            the sensor (e.g. velocity encoder) you are reading from
	 * @param output
	 *            the SpeedController you are telling what to do
	 */
	public VelocityPIDController(double kp, double ki, double kd, double kf, PIDSource source,
			SpeedControllerGroup output) {
		super(kp, ki, kd, kf, /* LinearDigitalFilter.singlePoleIIR(source, 0.1, Robot.rmap.getCycleTime()) */source,
				output, Robot.rmap.getCycleTime());
		out = output;
	}

	/**
	 * Sets the target speed
	 * 
	 * @param speed
	 *            the target speed in inches/second
	 */
	@Override
	public void pidWrite(double output) {
		set(output);
	}

	/**
	 * Sets the target speed
	 * 
	 * @param speed
	 *            the target speed in inches/second
	 */
	@Override
	public void set(double speed) {
		setSetpoint(speed);
	}

	public void setConsts(double kP, double kI, double kD, double kF) {
		super.setP(kP);
		super.setI(kI);
		super.setD(kD);
		super.setF(kF);
	}

	/**
	 * Gets the current setpoint sent to the PID
	 * 
	 * @return the current set setpoint/target/goal sent to the PID
	 */
	@Override
	public double get() {
		return getSetpoint();
	}

	/**
	 * Invert this side of the DT (flip forwards and backwards).
	 * 
	 * @param isInverted
	 *            invert this side of the DT or not
	 */
	@Override
	public void setInverted(boolean isInverted) {
		out.setInverted(isInverted);
	}

	/**
	 * Get whether or not this side of the DT is inverted.
	 * 
	 * @return is this side of the DT inverted or not
	 */
	@Override
	public boolean getInverted() {
		return out.getInverted();
	}

	/**
	 * Set the output to zero.
	 */
	@Override
	public void stopMotor() {
		out.stopMotor();
	}
}
