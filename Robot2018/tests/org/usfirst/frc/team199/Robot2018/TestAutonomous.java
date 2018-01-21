package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Position;
import org.usfirst.frc.team199.Robot2018.commands.Autonomous.Strategy;

import java.util.HashMap;
import java.util.Map;

class TestAutonomous {

	@Test
	void mansNotHot() {
		assertEquals(4, 2+2);
		assertEquals(3, 4-1);
	}
	
	@Test
	void test0() {
		Map<String, Autonomous.Strategy> strats = new HashMap<String, Autonomous.Strategy>();
		
		strats.put("LL", Strategy.NOTHING);
		strats.put("LR", Strategy.NOTHING);
		strats.put("RL", Strategy.NOTHING);
		strats.put("RR", Strategy.NOTHING);
		
		Position pos = Position.LEFT;
		String fmsInput = "LLL";
		double delay = 0;
		
		Autonomous testAuto = new Autonomous(pos, strats, delay, fmsInput);
		
		assertEquals("", testAuto.getScriptName());
		assertEquals(delay, testAuto.getDelay());
	}

	@Test
	void test1() {
		Map<String, Autonomous.Strategy> strats = new HashMap<String, Autonomous.Strategy>();
		
		strats.put("LL", Strategy.NOTHING);
		strats.put("LR", Strategy.AUTO_LINE);
		strats.put("RL", Strategy.NOTHING);
		strats.put("RR", Strategy.NOTHING);
		
		Position pos = Position.RIGHT;
		String fmsInput = "LRL";
		double delay = 0;
		
		Autonomous testAuto = new Autonomous(pos, strats, delay, fmsInput);
		
		assertEquals("Rxxx", testAuto.getScriptName());
		assertEquals(delay, testAuto.getDelay());
	}
	
	@Test
	void test2() {
		Map<String, Autonomous.Strategy> strats = new HashMap<String, Autonomous.Strategy>();
		
		strats.put("LL", Strategy.AUTO_LINE);
		strats.put("LR", Strategy.SCALE);
		strats.put("RL", Strategy.SWITCH_EXCHANGE);
		strats.put("RR", Strategy.SWITCH_SCALE);
		
		Position pos = Position.CENTER;
		String fmsInput = "RRL";
		double delay = 0;
		
		Autonomous testAuto = new Autonomous(pos, strats, delay, fmsInput);
		
		assertEquals("CRRx", testAuto.getScriptName());
		assertEquals(delay, testAuto.getDelay());
	}
	
	@Test
	void test3() {
		Map<String, Autonomous.Strategy> strats = new HashMap<String, Autonomous.Strategy>();
		
		strats.put("LL", Strategy.SWITCH);
		strats.put("LR", Strategy.SWITCH_EXCHANGE);
		strats.put("RL", Strategy.NOTHING);
		strats.put("RR", Strategy.SWITCH_SCALE);
		
		Position pos = Position.LEFT;
		String fmsInput = "LLR";
		double delay = 0;
		
		Autonomous testAuto = new Autonomous(pos, strats, delay, fmsInput);
		
		assertEquals("LLxx", testAuto.getScriptName());
		assertEquals(delay, testAuto.getDelay());
	}
	
	@Test
	void test4() {
		Map<String, Autonomous.Strategy> strats = new HashMap<String, Autonomous.Strategy>();
		
		strats.put("LL", Strategy.SWITCH);
		strats.put("LR", Strategy.SWITCH_EXCHANGE);
		strats.put("RL", Strategy.SWITCH_SCALE);
		strats.put("RR", Strategy.NOTHING);
		
		Position pos = Position.RIGHT;
		String fmsInput = "LRR";
		double delay = 0;
		
		Autonomous testAuto = new Autonomous(pos, strats, delay, fmsInput);
		
		assertEquals("RLxE", testAuto.getScriptName());
		assertEquals(delay, testAuto.getDelay());
	}
}
