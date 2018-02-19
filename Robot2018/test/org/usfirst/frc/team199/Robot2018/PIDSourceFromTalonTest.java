package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.usfirst.frc.team199.Robot2018.autonomous.PIDSourceFromTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSourceType;

class PIDSourceFromTalonTest {

	@Test
	void test1() {
		WPI_TalonSRX talon = mock(WPI_TalonSRX.class);

		when(talon.getControlMode()).thenReturn(ControlMode.Position);
		when(talon.getSelectedSensorPosition(0)).thenReturn(25);
		when(talon.getSelectedSensorVelocity(0)).thenReturn(5);

		PIDSourceFromTalon talSrc = new PIDSourceFromTalon(talon);

		assertEquals(talSrc.getPIDSourceType(), PIDSourceType.kDisplacement);
		assertEquals(talSrc.pidGet(), 25);

		talSrc.setPIDSourceType(PIDSourceType.kRate);
		assertEquals(talSrc.getPIDSourceType(), PIDSourceType.kRate);
		assertEquals(talSrc.pidGet(), 5);
	}

	@Test
	void test2() {
		WPI_TalonSRX talon = mock(WPI_TalonSRX.class);

		when(talon.getControlMode()).thenReturn(ControlMode.Velocity);
		when(talon.getSelectedSensorPosition(0)).thenReturn(25);
		when(talon.getSelectedSensorVelocity(0)).thenReturn(5);

		PIDSourceFromTalon talSrc = new PIDSourceFromTalon(talon);

		assertEquals(talSrc.getPIDSourceType(), PIDSourceType.kRate);
		assertEquals(talSrc.pidGet(), 5);

		talSrc.setPIDSourceType(PIDSourceType.kDisplacement);
		assertEquals(talSrc.getPIDSourceType(), PIDSourceType.kDisplacement);
		assertEquals(talSrc.pidGet(), 25);
	}

}
