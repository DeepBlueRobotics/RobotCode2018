package org.usfirst.frc.team199.Robot2018.autonomous;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceAverage implements PIDSource {

	private PIDSource lSrc;
	private PIDSource rSrc;
	private PIDSourceType type;

	public PIDSourceAverage(PIDSource leftSource, PIDSource rightSource) {
		lSrc = leftSource;
		rSrc = rightSource;
		if (leftSource.getPIDSourceType().equals(rightSource.getPIDSourceType())) {
			type = leftSource.getPIDSourceType();
		} else {
			throw new IllegalArgumentException();
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
		return (lSrc.pidGet() + rSrc.pidGet()) / 2;
	}

}
