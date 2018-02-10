package org.usfirst.frc.team199.Robot2018;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Map;

import org.junit.jupiter.api.Test;

import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;;

class ParseScriptFileTest {

	@Test
	void test0() {
		 Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile("asdf\nqwerty");
		 assertEquals(true, output.isEmpty());
	}

	@Test
	void test1() {
		String input = "asdf:\n"
				+ "move 43";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("move", output.get("asdf").get(0)[0]);
		assertEquals("43", output.get("asdf").get(0)[1]);
	}
	
	@Test
	void test2() {
		String input = "LxRE:\n"
				+ "turn 30";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("turn", output.get("LxRE").get(0)[0]);
		assertEquals("30", output.get("LxRE").get(0)[1]);
	}
	
	@Test
	void test3() {
		String input = "CLRx:\n"
				+ "turn (2,5)";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("turn", output.get("CLRx").get(0)[0]);
		assertEquals("(2,5)", output.get("CLRx").get(0)[1]);
	}
	
	@Test
	void test4() {
		String input = "LRxE:\n"
				+ "moveto 96";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("moveto", output.get("LRxE").get(0)[0]);
		assertEquals("96", output.get("LRxE").get(0)[1]);
	}
	
	@Test
	void test5() {
		String input = "CLxx:\n"
				+ "moveto (14,52)";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("moveto", output.get("CLxx").get(0)[0]);
		assertEquals("(14,52)", output.get("CLxx").get(0)[1]);
	}
	
	@Test
	void test6() {
		String input = "RxRx:\n"
				+ "moveto (4,20) (18,47) 39";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("moveto", output.get("RxRx").get(0)[0]);
		assertEquals("(4,20) (18,47) 39", output.get("RxRx").get(0)[1]);
	}
	
	@Test
	void test7() {
		String input = "Rxxx:\n"
				+ "wait 38";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("wait", output.get("Rxxx").get(0)[0]);
		assertEquals("38", output.get("Rxxx").get(0)[1]);
	}
	
	@Test
	void test8() {
		String input = "LxxE:\n"
				+ "exchange";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("exchange", output.get("LxxE").get(0)[0]);
		assertEquals("", output.get("LxxE").get(0)[1]);
	}
	
	@Test
	void test9() {
		String input = "RRxx:\n"
				+ "moveto (42,56) 45 # Trailing comment should not break anything \n"
				+ "scale\n"
				+ "#move 10\n" // For testing that commented lines are ignored
				+ "move 46\n" 
				+ "LRxx:\n"
				+ "intake\n"
				+ "turn (32,5)\n";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(2, output.size());
		assertEquals("moveto", output.get("RRxx").get(0)[0]);
		assertEquals("(42,56) 45", output.get("RRxx").get(0)[1]);
		assertEquals("scale", output.get("RRxx").get(1)[0]);
		assertEquals("", output.get("RRxx").get(1)[1]);
		assertEquals("move", output.get("RRxx").get(2)[0]);
		assertEquals("46", output.get("RRxx").get(2)[1]);
		
		assertEquals("intake", output.get("LRxx").get(0)[0]);
		assertEquals("", output.get("LRxx").get(0)[1]);
		assertEquals("turn", output.get("LRxx").get(1)[0]);
		assertEquals("(32,5)", output.get("LRxx").get(1)[1]);
	}
	
	@Test
	void test10() {
		String input = "CLRx:\n"
				+ "turn 0x4a"; // unfortunately, the program does not except hexadecimal xD
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals(true, output.get("CLRx").isEmpty());
	}
	
	@Test
	void test11() {
		String input = "CLRx:\n"
				+ "moveto (4,20 ) (18,47) 39";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals(true, output.get("CLRx").isEmpty());
	}
	
	@Test
	void test12() {
		String input = "CLRx:\n"
				+ "moveto (4,20)(18,47) 39";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals(true, output.get("CLRx").isEmpty());
	}
	
	@Test
	void test13() {
		String input = "CLRx:\n"
				+ "moveto (4 ,20) (18,47) 39";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals(true, output.get("CLRx").isEmpty());
	}
	
	@Test
	void test14() {
		String input = "LxxE:\n"
				+ " exchange    ";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("exchange", output.get("LxxE").get(0)[0]);
		assertEquals("", output.get("LxxE").get(0)[1]);
	}
	
	@Test
	void test15() {
		String input = "Rxxx:\n"
				+ "wait   38";
		Map<String, ArrayList<String[]>> output = AutoUtils.parseScriptFile(input);
		
		assertEquals(1, output.size());
		assertEquals("wait", output.get("Rxxx").get(0)[0]);
		assertEquals("38", output.get("Rxxx").get(0)[1]);
	}
}
