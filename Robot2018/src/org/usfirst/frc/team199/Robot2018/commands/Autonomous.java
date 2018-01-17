package org.usfirst.frc.team199.Robot2018.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Initially run during Auto. Responsible for getting input from SmartDashboard
 * and the FMS, and then calling RunAuto with the specified script.  
 */
public class Autonomous extends CommandGroup implements AutonomousInterface {

	/**
	 * The starting position of the robot relative to alliance member robots
	 */
	public enum Position  {
		LEFT ("Left", "L"),
		CENTER ("Center", "C"),
		RIGHT ("Right", "R")
		;
		
		private final String sdName;
		private final String shortName;
		Position (String sdName, String shortName) {
			this.sdName= sdName;
			this.shortName = shortName;
		}
		
		/**
		 * @return the name that should display on SmartDashboard
		 */
		public String getSDName() {
			return sdName;
		}
		
		/**
		 * @return the short name of the position used in the name of scripts
		 */
		public String getShortName() {
			return shortName;
		}
	}
	
	/**
	 * Strategy chosen by Manipulator before the match starts
	 */
	public enum Strategy {
		AUTO_LINE ("Cross Auto Line"),
		SWITCH ("Switch"),
		SCALE ("Scale"),
		EXCHANGE ("Exchange"),
		SWITCH_SCALE ("Switch and Scale"),
		SWITCH_EXCHANGE ("Switch and Exchange"),
		NOTHING ("Do nothing")
		;
		
		private final String sdName;
		Strategy (String sdName) {
			this.sdName = sdName;
		}
		
		/**
		 * @return the name that should display on SmartDashboard
		 */
		public String getSDName () {
			return sdName;
		}
	}
	
	/**
	 * Based on the input given, generates a unique script name and executes RunScript with it
	 * 
	 * @param startPos starting position of the robot, either Left, Center, or Right
	 * @param strategies a map of the chosen strategies (check robotInit in Robot.java for specifics), with keys being
	 * LL, LR, RR, and RL representing the switch and the scale locations respectively
	 * @param delay seconds of delay before executing the script. Used for coordinating with alliance partners 
	 * @param fmsInput A three character String representing the FMS input of the positions of our side switch, scale,
	 * and other side switch, respectively 
	 */
    public Autonomous(Position startPos, Map<String, Strategy> strategies, double delay, String fmsInput) {
    		String scriptName = "";
    		
    		scriptName += startPos.getShortName();
    		
    		Strategy chosenStrat = strategies.get(fmsInput.substring(0, 2));
    		
    		// skip the next steps if robot is chosen to do nothing
    		if (chosenStrat == Strategy.NOTHING)
    			return;
    		
    		// add switch, switch, and exchange location if going for them, "x" if not 
    		if (chosenStrat == Strategy.SWITCH || chosenStrat == Strategy.SWITCH_EXCHANGE || chosenStrat == Strategy.SWITCH_SCALE)
    			scriptName += fmsInput.substring(0, 1);
    		else
    			scriptName += "x";
    		
    		if (chosenStrat == Strategy.SCALE || chosenStrat == Strategy.SWITCH_SCALE)
    			scriptName += fmsInput.substring(1, 2);
    		else
    			scriptName += "x";
    		
    		if (chosenStrat == Strategy.EXCHANGE || chosenStrat == Strategy.SWITCH_EXCHANGE)
    			scriptName += "E";
    		else
    			scriptName += "x";
    		
    		addSequential(new WaitCommand(delay));
    		addSequential(new RunScript(scriptName));
    }
}
