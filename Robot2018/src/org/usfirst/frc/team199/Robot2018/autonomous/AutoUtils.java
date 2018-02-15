package org.usfirst.frc.team199.Robot2018.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class AutoUtils {

	public static Position position = new Position(0, 0, 0);

	/**
	 * Parses the inputted script file into a map of scripts
	 * 
	 * @param scriptFile
	 *            the script file to parse
	 * @return a map, with the key being the script name, and the argument being a
	 *         list of arrays that are instruction-argument pairs
	 */
	public static Map<String, ArrayList<String[]>> parseScriptFile(String scriptFile) {
		Map<String, ArrayList<String[]>> autoScripts = new HashMap<String, ArrayList<String[]>>();

		String lines[] = scriptFile.split("\\r?\\n");

		ArrayList<String[]> currScript = new ArrayList<String[]>();
		String currScriptName = "";

		int count = 1;
		for (String line : lines) {
			// remove comments
			int commentIndex = line.indexOf("#");
			if (commentIndex != -1)
				line = line.substring(0, commentIndex);

			// trim and remove extra whitespace just to make it neater
			line = line.trim().replaceAll("\\s+", " ");	
			// make coordinates with spaces also work
			int parenIndex = line.indexOf("(");
			while (parenIndex != -1) {
				// removes all spaces between the parentheses
				int endParenIndex = line.indexOf(")", parenIndex);
				String coord = line.substring(parenIndex + 1, endParenIndex);
				line = line.substring(0, parenIndex + 1) + coord.replaceAll(" ", "") + line.substring(endParenIndex);
				
				// finds next parentheses
				parenIndex = line.indexOf("(", parenIndex + 1);
			}
			// if there's no instruction on this line, skip
			if (line.equals("")) {
				continue;
			}

			// if current line is a label, store the previous script and make a new empty
			// one
			if (line.endsWith(":")) {
				autoScripts.put(currScriptName, currScript);
				currScript = new ArrayList<String[]>();
				currScriptName = line.substring(0, line.length() - 1);
			} else {

				// first separate the command into instruction and args
				String instruction;
				String args;

				int separator = line.indexOf(' ');
				if (separator == -1) {
					instruction = line;
					args = "";
				} else {
					instruction = line.substring(0, separator);
					args = line.substring(separator + 1);
				}

				// if it's valid, put it into the script
				if (isValidCommand(instruction, args, count)) {
					String[] command = { instruction, args };
					currScript.add(command);
				}
			}
			count++;
		}

		// puts the last script in
		autoScripts.put(currScriptName, currScript);

		// remove the stray one in the beginning
		autoScripts.remove("");

		return autoScripts;
	}

	/**
	 * Validates the command inputted to see if it's AAA compliant
	 * 
	 * @param instruction
	 *            the instruction/command name
	 * @param args
	 *            the arguments provided to the instruction. A blank String if none
	 * @param lineNumber
	 *            the lineNumber in the script file. used for logging warnings
	 * @return if the command is valid
	 */
	public static boolean isValidCommand(String instruction, String args, int lineNumber) {
		// moveto takes in a set of points, and the last arg can be a number
		if (instruction.equals("moveto")) {
			if (args == "") {
				logWarning(lineNumber, "The command `moveto` requires at least one argument.");
				return false;
			}

			String[] splitArgs = args.split(" ");
			for (int i = 0; i < splitArgs.length - 1; i++) {
				if (!isPoint(splitArgs[i])) {
					logWarning(lineNumber,
							"The arguments for command `moveto` should be points formatted like this: " + "`(x,y)`.");
					return false;
				}
			}

			if (!isDouble(splitArgs[splitArgs.length - 1]) && !isPoint(splitArgs[splitArgs.length - 1])) {
				logWarning(lineNumber, "The last argument for command `moveto` should be a number, or a point "
						+ "formatted like this: `(x,y)`.");
				return false;
			}
		}

		// turn can take a number or point
		else if (instruction.equals("turn")) {
			if (args.contains(" ")) {
				logWarning(lineNumber, "Command `turn` only accepts one argument.");
				return false;
			}

			if (!isDouble(args) && !isPoint(args)) {
				logWarning(lineNumber, "The argument for command `turn` should be a number or a point formatted like "
						+ "this: `(x,y)`.");
				return false;
			}
		}

		// move and wait can take only a number
		else if (instruction.equals("move") || instruction.equals("wait")) {
			if (args.contains(" ")) {
				logWarning(lineNumber, "Command `move` only accepts one argument.");
				return false;
			}

			if (!isDouble(args)) {
				logWarning(lineNumber, "The argument for command `move` should be a number.");
				return false;
			}
		}

		// switch, scale, exchange, intake, and end all don't have any args
		else if (instruction.equals("switch") || instruction.equals("scale") || instruction.equals("exchange")
				|| instruction.equals("intake") || instruction.equals("end")) {
			if (!args.equals("")) {
				logWarning(lineNumber, "Command `" + instruction + "` does not accept any arguments.");
				return false;
			}
		}

		// Jump only takes one argument
		else if (instruction.equals("jump")) {
			if (args.contains(" ")) {
				logWarning(lineNumber, "Command `jump` only accepts one argument.");
				return false;
			}
		}

		// if it's not even a valid instruction
		else {
			logWarning(lineNumber, "`" + instruction + "` is not a valid command.");
			return false;
		}

		// if everything is all good
		return true;
	}

	/**
	 * Helper method used by isValidCommand() to log warnings for non-valid
	 * commands.
	 * 
	 * @param lineNumber
	 *            the line number in the script file
	 * @param message
	 *            the message to log
	 */

	private static void logWarning (int lineNumber, String message) {
		System.err.println("[ERROR] Line " + lineNumber + ": " + message);
	}

	/**
	 * Helper method used by isValidCommand() to check if an argument is a point,
	 * characterized by parentheses on the left and right, with two numbers
	 * separated by a comma, with no whitespace in between.
	 * 
	 * @param s
	 *            the argument
	 * @return if the argument is a point
	 */
	public static boolean isPoint(String s) {
		// checks if it starts and ends with parentheses
		if (!s.startsWith("(") || !s.endsWith(")"))
			return false;

		// checks that there's one, and only one comma (like this phrase)
		int indexOfComma = s.indexOf(',');
		int count = 0;
		while (indexOfComma != -1) {
			count++;
			indexOfComma = s.indexOf(',', indexOfComma + 1);
		}
		if (count != 1)
			return false;

		// really ugly, but just checks if the stuff between the parentheses are numbers
		if (!isDouble(s.substring(1, s.indexOf(','))) || !isDouble(s.substring(s.indexOf(',') + 1, s.length() - 1)))
			return false;

		return true;
	}

	/**
	 * Helper method used by isValidCommand() used to check if an argument is able
	 * to be converted into a double
	 * 
	 * @param s
	 *            the argument
	 * @return if the argument is a double
	 */
	public static boolean isDouble(String s) {
		try {
			Double.parseDouble(s);
		} catch (Exception e) {
			return false;
		}
		return true;
	}

	public static double[] parsePoint(String cmdArgs) {
		double[] point = new double[2];
		String parentheseless;
		String[] pointparts;
		if (AutoUtils.isPoint(cmdArgs)) {
			parentheseless = cmdArgs.substring(1, cmdArgs.length() - 1);
			pointparts = parentheseless.split(",");
            try {
			    point[0] = Double.parseDouble(pointparts[0]);
			    point[1] = Double.parseDouble(pointparts[1]);
            } catch (Exception e) {
                point[0] = 1;
                point[1] = 1;
            }
		}
		return point;
	}
}
