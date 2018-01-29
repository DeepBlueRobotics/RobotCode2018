package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceAverage;

import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.HardwareTimer;

class PIDSourceAverageTest {

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
		PIDSource lEnc = mock(PIDSource.class);
		when(lEnc.getPIDSourceType()).thenReturn(PIDSourceType.kDisplacement);

		PIDSource rEnc = mock(PIDSource.class);
		when(rEnc.getPIDSourceType()).thenReturn(PIDSourceType.kDisplacement);
		PIDSourceAverage avg = new PIDSourceAverage(lEnc, rEnc);
		assertEquals(avg.getPIDSourceType(), PIDSourceType.kDisplacement);
	}

	@Test
	void test2() {
		PIDSource lEnc = mock(PIDSource.class);
		when(lEnc.getPIDSourceType()).thenReturn(PIDSourceType.kDisplacement);

		PIDSource rEnc = mock(PIDSource.class);
		when(rEnc.getPIDSourceType()).thenReturn(PIDSourceType.kRate);
		PIDSourceAverage avg;
		try {
			avg = new PIDSourceAverage(lEnc, rEnc);
		} catch (IllegalArgumentException e) {
			avg = new PIDSourceAverage(rEnc, rEnc);
		}
		assertEquals(avg.getPIDSourceType(), PIDSourceType.kRate);
	}

}
