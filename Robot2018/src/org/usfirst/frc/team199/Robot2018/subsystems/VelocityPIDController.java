package org.usfirst.frc.team199.Robot2018.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;

public class VelocityPIDController extends PIDController implements SpeedController {

	private SpeedController out;

	/**
	 * @param kp the proportional PID constant
	 * @param ki the integral PID constant
	 * @param kd the derivative PID constant
	 * @param source
	 *            the SpeedController you are reading from
	 * @param output
	 *            the SpeedController you are telling what to do
	 */
	public VelocityPIDController(double kp, double ki, double kd, PIDSource source, SpeedController output) {
		super(kp, ki, kd, source, output);
		out = output;
	}

	@Override
	public void pidWrite(double output) {
		setSetpoint(output);
	}

	@Override
	public void set(double speed) {
		// TODO Auto-generated method stub
		setSetpoint(speed);
	}

	@Override
	public double get() {
		// TODO Auto-generated method stub
		// should possibly be actual spdCtr value instead???
		return out.get();
	}

	@Override
	public void setInverted(boolean isInverted) {
		out.setInverted(isInverted);
	}

	@Override
	public boolean getInverted() {
		return out.getInverted();
	}

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		out.stopMotor();
	}
}
