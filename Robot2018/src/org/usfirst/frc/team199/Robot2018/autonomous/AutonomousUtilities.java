package org.usfirst.frc.team199.Robot2018.autonomous;

import java.util.ArrayList;
import java.util.Arrays;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Preferences;

public class AutonomousUtilities {
	private static boolean returnThis;
	public static void splitScriptFile() {
		String scriptFile = Preferences.getInstance().getString("autoscripts", null); //gets the string in autoscripts
		if (scriptFile == null) {
			//the script has failed
			returnThis = true; //make isFinished return true
		}
		ArrayList<String> lines = new ArrayList<String>(Arrays.asList(scriptFile.split("\\r?\\n"))); //splits the input from Preferences into lines and makes an ArrayList
		boolean finished = false; //new boolean to see if our test is finished
		ArrayList<ArrayList<String>> returned; //what is returned by colonSplit
		while (!finished) { //loops until we run out of colon lines
			returned = colonSplit(lines, 1); //fills returned with the result of calling colonSplit. first line is ignored, because the file is expected to begin with a line with a colon.
			finished = returned.get(1)==null;
			lines = returned.get(1);
			Robot.autoScripts.put(returned.get(0).get(0).substring(0, returned.get(0).get(0).length()-1), returned.get(0)); //sending scripts to Robot.java's hashmap
		}
		returnThis = true; //command is finished
	}
	private static ArrayList<ArrayList<String>> colonSplit(ArrayList<String> lines, int linesToIgnore) {
		ArrayList<String> current = new ArrayList<String>(); //this will be the ArrayList with the part before the first colon (it's initialized here because otherwise eclipse tells me it might not have been initialized)
		boolean finished = true; //this will be true if it can't find a colon where there should be one
		for (int i = linesToIgnore; i < lines.size(); i++) { //loop through the indices between the lines to ignore and the end of the array
			String line = lines.get(i); 
			finished = true;
			if (line.indexOf(":") > -1) {
				current = new ArrayList<String>(lines.subList(0, i));
				lines = new ArrayList<String>(lines.subList(i, lines.size()));
				finished = false;
			}
		}
		if (finished) { //runs if the script doesn't split
			current = lines;
			lines = null;
		}
		ArrayList<ArrayList<String>> done = new ArrayList<ArrayList<String>>();
		done.add(current);
		done.add(lines);
		return done;
	}
}
