package org.usfirst.frc.team199.Robot2018.commands;

import java.awt.Robot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Gets the script name and runs the script.
 */
public class RunScript extends CommandGroup implements RunScriptInterface {

    public RunScript(String scriptName) {
    		ArrayList<String> script = Robot.autoScripts.getOrDefault(scriptName, new ArrayList<String>());
    		
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
	    				addSequential(new AutoTurn(cmdArgs[0]));
	    				break;
	    			case "move":
	    				addSequential(new AutoMove(cmdArgs[0]));
	    				break;
	    			case "switch":
	    				addSequential(new DeployToSwitch());
	    				break;
	    			case "scale":
	    				addSequential(new DeployToScale());
	    				break;
	    			case "exchange":
	    				addSequential(new DeployToExchange);
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
	    				throw new Exception();
    			}
    		}
    }
}
