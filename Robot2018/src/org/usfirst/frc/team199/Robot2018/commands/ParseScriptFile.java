package org.usfirst.frc.team199.Robot2018.commands;

import java.io.ObjectInputStream.GetField;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import javax.security.auth.login.FailedLoginException;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.hal.ThreadsJNI;

/**
 *
 */
public class ParseScriptFile extends Command {

    public ParseScriptFile() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		String scriptFile = Preferences.getInstance().getString("autoscripts", "aaaaaaa");
		ArrayList<String> lines = new ArrayList<String>(Arrays.asList(scriptFile.split("\\r?\\n")));
		boolean finished = false;
		ArrayList<ArrayList<String>> returned;
		while (!finished) {
			returned = splitAtSecondColon(lines);
			finished = returned.get(1)==null;
			lines = returned.get(1);
			Robot.autoScripts.put(returned.get(0).get(0).substring(0, returned.get(0).get(0).length()-1), returned.get(0));
		}
    }
	private ArrayList<ArrayList<String>> splitAtSecondColon(ArrayList<String> lines) {
		ArrayList<String> current = new ArrayList<String>();
		boolean finished = true;
		for (int i = 0; i < lines.size(); i++) {
			String line = lines.get(i);
			finished = true;
			if (line.indexOf(":") > -1 && i != 0) {
				current = new ArrayList<String>(lines.subList(0, i));
				lines = new ArrayList<String>(lines.subList(i, lines.size()));
				finished = false;
			}
		}
		if (finished) {
			current = lines;
			lines = null;
		}
		ArrayList<ArrayList<String>> done = new ArrayList<ArrayList<String>>();
		done.add(current);
		done.add(lines);
		return done;
	}
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
