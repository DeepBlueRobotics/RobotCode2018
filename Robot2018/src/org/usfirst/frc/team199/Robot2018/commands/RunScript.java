package org.usfirst.frc.team199.Robot2018.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Gets the script name and interprets the script into individual commands.
 */
public class RunScript extends CommandGroup implements RunScriptInterface {

    public RunScript(String scriptName) {
    		// make sure to uncomment this when autoScripts is written
    		ArrayList<String> script = new ArrayList<String>(); // Robot.autoScripts.getOrDefault(scriptName, new ArrayList<String>());
    		
    		outerloop:
    		for(String cmd : script) {
    			String[] cmdParts = cmd.split(" ");
    			String cmdName = cmdParts[0];
    			String[] cmdArgs = Arrays.copyOfRange(cmdParts, 1, cmdParts.length);
    			
    			switch (cmdParts[0]) {
	    			case "moveto":
	    				addSequential(new AutoMoveTo(cmdArgs));
	    				break;
	    			case "turn":
	    				addSequential(new AutoTurn(Double.parseDouble(cmdArgs[0])));
	    				break;
	    			case "move":
	    				addSequential(new AutoMove(Double.parseDouble(cmdArgs[0])));
	    				break;
	    			case "switch":
	    				addSequential(new EjectToSwitch());
	    				break;
	    			case "scale":
	    				addSequential(new EjectToScale());
	    				break;
	    			case "exchange":
	    				addSequential(new EjectToExchange());
	    				break;
	    			case "wait":
	    				addSequential(new WaitCommand(Double.parseDouble(cmdArgs[0])));
	    				break;
	    			case "intake":
	    				addSequential(new IntakeCube());
	    				break;
	    			case "jump":
	    				addSequential(new RunScript(cmdArgs[0]));
	    				break;
	    			case "end":
	    				break outerloop;
	    			default:
	    				System.out.println("`" + cmdParts[0] + "`" + " is not a valid command name. Check AAA Reference.");
    			}
    		}
    }
}
