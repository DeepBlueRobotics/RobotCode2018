package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.DashboardInterface;

public interface TestAutoInterface extends DashboardInterface {
	/**
	 * gets the script name that we want to run from SmartDashboard
	 * 
	 * @return the script name
	 */
	public String getScriptToTest();
}
