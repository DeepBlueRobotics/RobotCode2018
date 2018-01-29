package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.usfirst.frc.team199.Robot2018.autonomous.VelocityPIDController;

import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.HardwareTimer;

class VelocityPIDControllerTest {

	@BeforeEach
	void setUp() {
		// Since VelocityPIDController extends PIDController and PIDController calls
		// static methods in wpilib that only work on robot,
		// we setup these mocks to allow the tests to run off robot.
		HardwareTimer tim = mock(HardwareTimer.class);
		Timer.Interface timerInstance = mock(Timer.Interface.class);
		when(tim.newTimer()).thenReturn(timerInstance);
		Timer.SetImplementation(tim);
		HLUsageReporting.Interface usageReporter = mock(HLUsageReporting.Interface.class);
		HLUsageReporting.SetImplementation(usageReporter);
	}

	@Test
	void test1() {
		PIDSource source = mock(PIDSource.class);
		SpeedControllerGroup out = mock(SpeedControllerGroup.class);
		double p = 1;
		double i = 0.5;
		double d = 0.0037;
		VelocityPIDController vPID = new VelocityPIDController(p, i, d, source, out);

		vPID.set(20);
		assertEquals(vPID.get(), 20);
	}

	@Test
	void test2() {
		PIDSource source = mock(PIDSource.class);
		SpeedControllerGroup out = mock(SpeedControllerGroup.class);
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
		SpeedControllerGroup out = mock(SpeedControllerGroup.class);
		double p = 1;
		double i = 0.5;
		double d = 0.0037;
		VelocityPIDController vPID = new VelocityPIDController(p, i, d, source, out);

		vPID.set(20);
		assertEquals(vPID.getSetpoint(), 20);
	}

}
