package org.usfirst.frc.team199.Robot2018.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceFromTalon implements PIDSource {

	private PIDSourceType type;
	private WPI_TalonSRX talon;

	public PIDSourceFromTalon(WPI_TalonSRX talon) {
		this.talon = talon;
		ControlMode mode = talon.getControlMode();
		switch (mode.toString()) {
		case "Position":
			type = PIDSourceType.kDisplacement;
			break;
		default:
			type = PIDSourceType.kRate;
			break;
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		switch (type.toString()) {
		case "kDisplacement":
			return talon.getSelectedSensorPosition(0);
		default:
			return talon.getSelectedSensorVelocity(0);
		}
	}

}
