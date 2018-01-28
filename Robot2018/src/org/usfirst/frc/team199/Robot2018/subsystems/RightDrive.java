package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class RightDrive implements PIDOutput, PIDSource {

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return Robot.dt.getRightEnc();
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
	}

	@Override
	public void pidWrite(double arg0) {
		Robot.dt.pidRightWrite(arg0);
	}

}
