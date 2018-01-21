package org.usfirst.frc.team199.Robot2018.autonomous;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.usfirst.frc.team199.Robot2018.Robot;

public class TestAutoUtils {
	@Test
	void testColonSplitter() {
		ArrayList<String> testedlist = new ArrayList<String>();
		testedlist.add("this is a string with a colon :");
		testedlist.add("this is a string without a colon a");
		testedlist.add("this is a string without a colon b");
		testedlist.add("this is a string without a colon c");
		testedlist.add("this is a string with a colon :");
		testedlist.add("this is a string without a colon");
		testedlist.add("this is a string without a colon");
		testedlist.add("this is a string without a colon");
		
		ArrayList<ArrayList<String>> expected = new ArrayList<ArrayList<String>>();
		ArrayList<String> expected0 = new ArrayList<String>();
		ArrayList<String> expected1 = new ArrayList<String>();
		expected0.add("this is a string with a colon :");
		expected0.add("this is a string without a colon a");
		expected0.add("this is a string without a colon b");
		expected0.add("this is a string without a colon c");
		expected1.add("this is a string with a colon :");
		expected1.add("this is a string without a colon");
		expected1.add("this is a string without a colon");
		expected1.add("this is a string without a colon");
		expected.add(expected0);
		expected.add(expected1);
		
		assertEquals(expected, AutonomousUtilities.colonSplit(testedlist, 1));
		//System.out.println(expected.get(0).get(0));
	}
	
	@Test
	void testSplitScript() {
		Map<String, ArrayList<String>> expected = new HashMap<String, ArrayList<String>>();
		String stringToTest = "1:\n1-1\n1-2\n1-3\n2:\n2-1\n2-2\n2-3\n3:\n3-1\n2-3\n3-3\n";
		ArrayList<String> arr1 = new ArrayList<String>(Arrays.asList(new String[] {"1-1", "1-2", "1-3"}));
		ArrayList<String> arr2 = new ArrayList<String>(Arrays.asList(new String[] {"2-1", "2-2", "2-3"}));
		ArrayList<String> arr3 = new ArrayList<String>(Arrays.asList(new String[] {"3-1", "3-2", "3-3"}));
		
		expected.put("1:", arr1);
		expected.put("2:", arr2);
		expected.put("3:", arr3);
		
		AutonomousUtilities.splitScriptFile(stringToTest);
		
		assertEquals(expected, Robot.autoScripts);
	}
}




