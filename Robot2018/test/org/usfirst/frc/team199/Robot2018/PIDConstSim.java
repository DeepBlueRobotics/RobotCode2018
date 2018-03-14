package org.usfirst.frc.team199.Robot2018;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.runner.RunWith;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;

import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Arrays;
import java.util.Collection;

import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.HardwareTimer;
import org.junit.runners.Parameterized;
import org.junit.runners.Parameterized.Parameters;
import org.junit.runners.Parameterized.Parameter;

//@RunWith(value = Parameterized.class)
class PIDConstSim {
	static final double initkP = 10;
	static final double initkI = 6;
	static final double initkD = 6;
	
	//@Parameter(value = )
	double liftkP = initkP;
	double liftkI = initkI;
	double liftkD = initkD;
	
	private final double targetOne = 24;
	private final double targetTwo = 60;
	private final double targetThree = 45;
	// 1 in in meters
	private final double tolerance = 0.0254;
	
	double position = 0;
	double velocity_ = 0;
	double voltage_ = 0;
	double offset_ = -0.1;
	
	// Stall Torque in N m
	static final double kStallTorque = 2.41;
	// Stall Current in Amps
	static final double kStallCurrent = 131;
	// Free Speed in RPM
	static final double kFreeSpeed = 5330;
	// Free Current in Amps
	static final double kFreeCurrent = 2.7;
	// Mass of the Elevator
	// 51.3 lbs
	static final double kMass = 23.2693;
	// Number of motors
	static final double kNumMotors = 3.0;
	// Resistance of the motor
	static final double kResistance = 12.0 / kStallCurrent;
	// Motor velocity constant
	static final double Kv = ((kFreeSpeed / 60.0 * 2.0 * Math.PI) /
	             (12.0 - kResistance * kFreeCurrent));
	// Torque constant
	static final double Kt = (kNumMotors * kStallTorque) / kStallCurrent;
	// Gear ratio
	// 11.754 lift gear ratio reduction?
	static final double kG = 11.754;
	// Radius of pulley
	static final double kr = 0.02273;

	// Control loop time step
	static final double kDt = 0.01;
	    
	double current_time = 0;
	
	  // V = I * R + omega / Kv
	  // torque = Kt * I
	
	
	// (name = "{index}: testAdd({0}+{1}) = {2}")
	/*@Parameters
    public static double[][] data() {
    	// array is P I and D finalants in order
        //double [][][] PIDfinalTable = new double[5][5][5];
		double [][] PIDfinalTable = new double[3][5];
        double diffInter3 = initkD / 10;
        double diffInter2 = initkI / 10;
        double diffInter1 = initkP / 10;
        for(int i = 0; i < 3; i++) {
        	for(int j = -2; j < 3; j++) {
        		double res;
        		if(i == 0)
        			res = initkP + j * diffInter1;
        		else if(i == 1)
        			res = initkI + j * diffInter2;
        		else
        			res = initkD + j * diffInter3;
        		PIDfinalTable[i][j] = res;
        	}
        }
    	//return (Iterable<Double[]>)Arrays.asList(PIDfinalTable);
        return PIDfinalTable;
    }
    */
	
	private double GetAcceleration(double voltage) {
	    return -Kt * kG * kG / (Kv * kResistance * kr * kr * kMass) * velocity_ +
	           kG * Kt / (kResistance * kr * kMass) * voltage;
	  }
	
	private void simulateTime(double time, double voltage) {
		final double starting_time = time;
	    while (time > 0) {
	      final double current_dt = Math.min(time, 0.001);
	      //System.out.println("vel before: " + velocity_);
	      position += current_dt * velocity_;
	      double acc = GetAcceleration(voltage);
	      //System.out.println("acc: " + acc);
	      velocity_ += current_dt * acc;
	      //System.out.println("vel after: " + velocity_);
	      time -= 0.001;
	      //EXPECT_LE(position, ElevatorLoop::kMaxHeight + 0.01);
	      //EXPECT_GE(position, ElevatorLoop::kMinHeight - 0.01);
	    }
	    current_time += starting_time;
	}
	
	@BeforeEach
	void setUp() {
		// Since VelocityPIDController extends PIDController and PIDController calls
		// static methods in wpilib that only work on robot,
		// we setup these mocks to allow the tests to run off robot.
		// can't start at 0 because it will crash timer
		current_time = kDt;
		HardwareTimer tim = mock(HardwareTimer.class);
		Timer.Interface timerInstance = mock(Timer.Interface.class);
		when(tim.newTimer()).thenReturn(timerInstance);
		when(timerInstance.get()).thenAnswer(
			     new Answer() {
			         public Object answer(InvocationOnMock invocation) {
			             //return getTime();
			        	 return current_time;
			         }
			 });
		
		Timer.SetImplementation(tim);
		HLUsageReporting.Interface usageReporter = mock(HLUsageReporting.Interface.class);
		HLUsageReporting.SetImplementation(usageReporter);
	}
	
	class PIDSim{
		public PIDSim() {
			//position = 0;
		}
		
		public double pidGet() {
			return position;
		}
		
		public void pidWrite(double output) {
			//System.out.println("voltage: " + output);
			simulateTime(kDt, output * 12);
			System.out.println("position: " + position);
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

	void test24() {
		PIDSim sim = new PIDSim();
		PIDSimSource src = new PIDSimSource(sim);
		src.setPIDSourceType(PIDSourceType.kDisplacement);
		PIDSimOutput out = new PIDSimOutput(sim);
		PIDSimController liftController = new PIDSimController(liftkP, liftkI, liftkD, src, out);
		liftController.setSetpoint(0.6);
		liftController.setOutputRange(-1.0, 1.0);
		liftController.enable();
		//1 second?
		for(int i = 0; i < 100; i++) {
			liftController.calc();
		}
		System.out.println("Test" + targetOne + ": " + sim.pidGet());
		System.out.println("curr_time_1: " + current_time);
		assertTrue(sim.pidGet() > 0);
		assertTrue(Math.abs(0.6 - sim.pidGet()) < tolerance);
	}

	void test60() {
		PIDSim sim = new PIDSim();
		PIDSimSource src = new PIDSimSource(sim);
		src.setPIDSourceType(PIDSourceType.kDisplacement);
		PIDSimOutput out = new PIDSimOutput(sim);
		PIDSimController liftController = new PIDSimController(liftkP, liftkI, liftkD, src, out);
		liftController.setSetpoint(1.524);
		liftController.setOutputRange(-1.0, 1.0);
		liftController.enable();
		//1 second?
		for(int i = 0; i < 200; i++) {
			liftController.calc();
		}
		System.out.println("" + sim.pidGet());
		assertTrue(sim.pidGet() > 0);
		assertTrue(Math.abs(1.524 - sim.pidGet()) < tolerance);
	}

	@Test
	void test45() {
		PIDSim sim = new PIDSim();
		PIDSimSource src = new PIDSimSource(sim);
		src.setPIDSourceType(PIDSourceType.kDisplacement);
		PIDSimOutput out = new PIDSimOutput(sim);
		PIDSimController liftController = new PIDSimController(liftkP, liftkI, liftkD, src, out);
		liftController.setSetpoint(1.143);
		liftController.setOutputRange(-1.0, 1.0);
		liftController.enable();
		//1 second?
		for(int i = 0; i < 100; i++) {
			liftController.calc();
		}
		System.out.println("" + sim.pidGet());
		assertTrue(sim.pidGet() > 0);
		assertTrue(Math.abs(1.143 - sim.pidGet()) < tolerance);
	}

}
