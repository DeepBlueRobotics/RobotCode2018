package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.usfirst.frc.team199.Robot2018.autonomous.VelocityPIDController;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.HardwareTimer;

class VelocityPIDControllerTest {

	@Test
	void test1() {
		HardwareTimer tim = new HardwareTimer();
		Timer.SetImplementation(tim);
		PIDSource source = mock(PIDSource.class);
		SpeedController out = mock(SpeedController.class);
		double p = 1;
		double i = 0.5;
		double d = 0.0037;
		VelocityPIDController vPID = new VelocityPIDController(p, i, d, source, out);

		when(out.get()).thenReturn(6.5);
		assertEquals(6.5, vPID.get());
		verify(out).get();
	}

	@Test
	void test2() {
		PIDSource source = mock(PIDSource.class);
		SpeedController out = mock(SpeedController.class);
		double p = 1;
		double i = 0.5;
		double d = 0.0037;
		VelocityPIDController vPID = new VelocityPIDController(p, i, d, source, out);

		vPID.pidWrite(20);
		assertEquals(vPID.getSetpoint(), 20);
	}

	@Test
	void test3() {
		PIDSource source = mock(PIDSource.class);
		SpeedController out = mock(SpeedController.class);
		double p = 1;
		double i = 0.5;
		double d = 0.0037;
		VelocityPIDController vPID = new VelocityPIDController(p, i, d, source, out);

		vPID.set(20);
		assertEquals(vPID.getSetpoint(), 20);
	}

}
