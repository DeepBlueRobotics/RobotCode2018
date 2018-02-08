package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.internal.HardwareTimer;
import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team199.Robot2018.commands.Autonomous;
import org.usfirst.frc.team199.Robot2018.commands.TeleopDrive;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Position;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Strategy;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;
import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
import org.usfirst.frc.team199.Robot2018.commands.AutoMoveTo;

import java.util.HashMap;
import java.util.Map;

class AutoMoveToTest {

	// May need a BeforeEach to reset AutoUtils.

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
	void testWPICommand() {
		Command command = new Command() {
			protected boolean isFinished() { return false; }			
		};
		assertNotNull(command);
	}
	
	@Test
	void testWPICommandGroup() {
		CommandGroup group = new CommandGroup();
		assertNotNull(group);

		Command command = new Command() {
			protected boolean isFinished() { return false; }			
		};
		group.addSequential(command);
	}
	
	@Test
	void testPIDController() {
		PIDSource source = mock(PIDSource.class);
		PIDOutput output = mock(PIDOutput.class);		
		
		PIDController ctrl = new PIDController(0, 0, 0, source, output);
		assertNotNull(ctrl);
	}
	
	//@Test
	// Problem instantiating Subsystem because of SendableBase using network tables.
	void testWPISubsystem() {
		//LiveWindow.setEnabled(false);
		Subsystem subsystem = new Subsystem() {
			protected void initDefaultCommand() {
				setDefaultCommand(new Command() {
					protected boolean isFinished() { return false; }			
				});
			}
		};
		assertNotNull(subsystem);
	}
		
	@Test
	void testForwardAndRight() {
		String[] args = {"(0,12)","(12,12)"};
		
		AutoUtils.setRot(0);
		AutoUtils.setX(0);
		AutoUtils.setY(0);
		
		PIDSource pidGyroSrc = mock(PIDSource.class);
		when(pidGyroSrc.getPIDSourceType()).thenReturn(PIDSourceType.kDisplacement);
		DrivetrainInterface dt = mock(DrivetrainInterface.class);
		when(dt.getGyro()).thenReturn(pidGyroSrc);		
		SmartDashboardInterface sd = mock(SmartDashboardInterface.class);
		PIDSource pidMoveSrc = mock(PIDSource.class);
		
		AutoMoveTo testAMT = new AutoMoveTo(args, dt, sd, pidMoveSrc);
		
		assertEquals(90, AutoUtils.getRot());
		assertEquals(12, AutoUtils.getX());
		assertEquals(12, AutoUtils.getY());
	}

	@Test
	void testForward() {
		String[] args = {"(0,12)"};
		
		AutoUtils.setRot(0);
		AutoUtils.setX(0);
		AutoUtils.setY(0);
		
		PIDSource pidGyroSrc = mock(PIDSource.class);
		when(pidGyroSrc.getPIDSourceType()).thenReturn(PIDSourceType.kDisplacement);
		DrivetrainInterface dt = mock(DrivetrainInterface.class);
		when(dt.getGyro()).thenReturn(pidGyroSrc);		
		SmartDashboardInterface sd = mock(SmartDashboardInterface.class);
		PIDSource pidMoveSrc = mock(PIDSource.class);
		
		AutoMoveTo testAMT = new AutoMoveTo(args, dt, sd, pidMoveSrc);
		
		assertEquals(0, AutoUtils.getRot());
		assertEquals(0, AutoUtils.getX());
		assertEquals(12, AutoUtils.getY());
	}
}
