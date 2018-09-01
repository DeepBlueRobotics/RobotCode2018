package org.usfirst.frc.team199.Robot2018.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;

public class TalonVelocityController implements SpeedController {

	private WPI_TalonSRX talon;

	/**
	 * Constructs a TalonVelocityContoller. Sets the four PID constants and the
	 * FeedbackDevice of the output Talon.
	 * 
	 * @param kp
	 *            the proportional PID constant, multiplied by the error
	 * @param ki
	 *            the integral PID constant, multiplied by the total error
	 * @param kd
	 *            the derivative PID constant, multiplied by the change in error
	 * @param kf
	 *            the feed forward value: should be 1/MaxSpeed
	 * @param source
	 *            the sensor (e.g. velocity encoder) you are reading from
	 * @param output
	 *            the Talon SRX you are telling what to do
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no blocking
	 *            or checking is performed.
	 */
	public TalonVelocityController(double kp, double ki, double kd, double kf, PIDSource source, WPI_TalonSRX output,
			int slotIdx, int timeoutMs) {
		talon = output;
		talon.config_kP(slotIdx, kp, timeoutMs);
		talon.config_kI(slotIdx, ki, timeoutMs);
		talon.config_kD(slotIdx, kd, timeoutMs);
		talon.config_kF(slotIdx, kf, timeoutMs);
		if (source instanceof Encoder && source.getPIDSourceType().equals(PIDSourceType.kRate)) {
			talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeoutMs);
		} else {
			throw new IllegalArgumentException();
		}
	}

	/**
	 * Sets the output of the output Talon based on the specified Velocity
	 * ControlMode.
	 * 
	 * @param output
	 *            setpoint value in inches/second
	 */
	@Override
	public void pidWrite(double output) {
		set(output);
	}

	/**
	 * Sets the output of the output Talon based on the specified Velocity
	 * ControlMode.
	 * 
	 * @param output
	 *            setpoint value in inches/second
	 */
	@Override
	public void set(double output) {
		// the div. by 10 is to translate from our desired param units (in/s) to the
		// Talon's set() method param units (in/100ms)
		talon.set(ControlMode.Velocity, output / 10);
	}

	/**
	 * Get the current set speed of this side of the DT.
	 * 
	 * @return the current set speed of this side of the DT from [-1, 1]
	 */
	@Override
	public double get() {
		return talon.get();
	}

	/**
	 * Sets the kP constant.
	 * 
	 * @param value
	 *            the value to set kP to
	 */
	public void setP(double value) {
		talon.config_kP(0, value, 0);
	}

	/**
	 * Sets the kI constant.
	 * 
	 * @param value
	 *            the value to set kI to
	 */
	public void setI(double value) {
		talon.config_kI(0, value, 0);
	}

	/**
	 * Sets the kD constant.
	 * 
	 * @param value
	 *            the value to set kD to
	 */
	public void setD(double value) {
		talon.config_kD(0, value, 0);
	}

	/**
	 * Sets the feed forward gain.
	 * 
	 * @param value
	 *            the value to set kF to
	 */
	public void setF(double value) {
		talon.config_kF(0, value, 0);
	}

	/**
	 * Invert this side of the DT (flip forwards and backwards).
	 * 
	 * @param isInverted
	 *            invert this side of the DT or not
	 */
	@Override
	public void setInverted(boolean isInverted) {
		talon.setInverted(isInverted);
	}

	/**
	 * Get whether or not this side of the DT is inverted.
	 * 
	 * @return is this side of the DT inverted or not
	 */
	@Override
	public boolean getInverted() {
		return talon.getInverted();
	}

	/**
	 * Disable this controller.
	 */
	@Override
	public void disable() {
		talon.disable();
	}

	/**
	 * Stop the motor until Set is called again.
	 */
	@Override
	public void stopMotor() {
		talon.stopMotor();
	}

	/**
	 * Gets the selected sensor position (in raw sensor units).
	 * 
	 * @return the selected sensor position (in raw sensor units).
	 */
	public double getCurrentPosition() {
		return talon.getSelectedSensorPosition(0);
	}

	/**
	 * Gets the selected sensor velocity (in raw sensor units per 100ms).
	 * 
	 * @return the selected sensor velocity (in raw sensor units per 100ms).
	 */
	public double getCurrentVelocity() {
		return talon.getSelectedSensorVelocity(0);
	}

}
