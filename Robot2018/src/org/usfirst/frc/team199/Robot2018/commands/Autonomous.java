package org.usfirst.frc.team199.Robot2018.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Initially run during Auto. Responsible for getting input from SmartDashboard
 * and the FMS, and then calling RunAuto with the specified script.  
 */
public class Autonomous extends CommandGroup implements AutonomousInterface {

    public Autonomous(String startPos, Map<String, String> strategies, double delay, String fmsInput) {
    		String scriptName = "";
    		
    		scriptName += startPos.substring(0, 1);
    		
    		String chosenStrat = strategies.get(fmsInput.substring(0, 2));
    		
    		// skip the next steps if robot is chosen to do nothing
    		if (chosenStrat.equals("Do nothing"))
    			return;
    		
    		// add switch, switch, and exchange location if going for them, "x" if not 
    		if (chosenStrat.contains("Switch"))
    			scriptName += fmsInput.substring(0, 1);
    		else
    			scriptName += "x";
    		
    		if (chosenStrat.contains("Scale"))
    			scriptName += fmsInput.substring(1, 2);
    		else
    			scriptName += "x";
    		
    		if (chosenStrat.contains("Exchange"))
    			scriptName += "E";
    		else
    			scriptName += "x";
    		
    		addSequential(new WaitCommand(delay));
    		addSequential(new RunScript(scriptName));
    }
}
