package org.usfirst.frc.team199.Robot2018;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.HardwareTimer;

class AutoLiftTest {
	
	final double liftkP = 0.01;
	final double liftkI = 0.0001;
	final double liftkD = 0.002;
	private final double targetOne = 24;
	private final double targetTwo = 60;
	private final double targetThree = 45;
	private int iterations;
	private final double tolerance = 1;
	
	@BeforeEach
	void setUp() {
		// Since VelocityPIDController extends PIDController and PIDController calls
		// static methods in wpilib that only work on robot,
		// we setup these mocks to allow the tests to run off robot.
		iterations = 0;
		HardwareTimer tim = mock(HardwareTimer.class);
		Timer.Interface timerInstance = mock(Timer.Interface.class);
		when(tim.newTimer()).thenReturn(timerInstance);
		when(timerInstance.get()).thenAnswer(
			     new Answer() {
			         public Object answer(InvocationOnMock invocation) {
			             return getTime();
			         }
			 });
		
		Timer.SetImplementation(tim);
		HLUsageReporting.Interface usageReporter = mock(HLUsageReporting.Interface.class);
		HLUsageReporting.SetImplementation(usageReporter);
	}
	
	private double getTime() {
		iterations++;
		return iterations * 0.02;
	}
	
	class PIDSim{
		private double totalDist = 0;
		private final double distPerPulse = 0.01;
		// 1/pulses per rev, 1/99?
		private final double circum = Math.PI * 6;
		public PIDSim() {
			totalDist = 0;
		}
		
		public double pidGet() {
			return totalDist;
		}
		
		public void pidWrite(double output) {
			//256 rpm
			totalDist += (output * 256) * distPerPulse * circum;
		}
	}
	
	class PIDSimController extends PIDController{
		
		public PIDSimController(double p, double i, double d, PIDSource src, PIDOutput out) {
			super(p,i,d,src,out);
		}
		
		public void calc() {
			this.calculate();
		}
		
	}
	
	class PIDSimSource implements PIDSource{
		private PIDSim sim;
		private PIDSourceType type;
		public PIDSimSource(PIDSim sim) {
			this.sim = sim;
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return type;
		}

		@Override
		public double pidGet() {
			return sim.pidGet();
		}
		
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			type = pidSource;
		}
	}
	
	class PIDSimOutput implements PIDOutput{
		private PIDSim sim;
		public PIDSimOutput(PIDSim sim) {
			this.sim = sim;
		}
		@Override
		public void pidWrite(double output) {
			sim.pidWrite(output);
		}
	}

	@Test
	void test24() {
		PIDSim sim = new PIDSim();
		PIDSimSource src = new PIDSimSource(sim);
		src.setPIDSourceType(PIDSourceType.kDisplacement);
		PIDSimOutput out = new PIDSimOutput(sim);
		PIDSimController liftController = new PIDSimController(liftkP, liftkI, liftkD, src, out);
		liftController.setSetpoint(targetOne);
		liftController.setOutputRange(-1.0, 1.0);
		liftController.enable();
		//1 second?
		for(int i = 0; i < 50; i++) {
			liftController.calc();
		}
		System.out.println("" + sim.pidGet());
		assert(sim.pidGet() > 0);
		assert(Math.abs(targetOne - sim.pidGet()) < tolerance);
	}
	
	@Test
	void test60() {
		PIDSim sim = new PIDSim();
		PIDSimSource src = new PIDSimSource(sim);
		src.setPIDSourceType(PIDSourceType.kDisplacement);
		PIDSimOutput out = new PIDSimOutput(sim);
		PIDSimController liftController = new PIDSimController(liftkP, liftkI, liftkD, src, out);
		liftController.setSetpoint(targetTwo);
		liftController.setOutputRange(-1.0, 1.0);
		liftController.enable();
		//1 second?
		for(int i = 0; i < 50; i++) {
			liftController.calc();
		}
		System.out.println("" + sim.pidGet());
		assert(sim.pidGet() > 0);
		assert(Math.abs(targetTwo - sim.pidGet()) < tolerance);
	}
	
	@Test
	void test45() {
		PIDSim sim = new PIDSim();
		PIDSimSource src = new PIDSimSource(sim);
		src.setPIDSourceType(PIDSourceType.kDisplacement);
		PIDSimOutput out = new PIDSimOutput(sim);
		PIDSimController liftController = new PIDSimController(liftkP, liftkI, liftkD, src, out);
		liftController.setSetpoint(targetThree);
		liftController.setOutputRange(-1.0, 1.0);
		liftController.enable();
		//1 second?
		for(int i = 0; i < 50; i++) {
			liftController.calc();
		}
		System.out.println("" + sim.pidGet());
		assert(sim.pidGet() > 0);
		assert(Math.abs(targetThree - sim.pidGet()) < tolerance);
	}

}
