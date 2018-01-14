package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.DashboardInterface;

import edu.wpi.first.wpilibj.DriverStation;

public interface AutonomousInterface extends DashboardInterface{
	public enum Position {
		LEFT,
		CENTER,
		RIGHT
	}
	
	public enum Strategy {
		AUTO_LINE,
		SWITCH,
		SCALE,
		EXCHANGE,
		SWITCH_SCALE,
		SWITCH_EXCHANGE,
		NOTHING
	}
	
	/**
	 * Gets the starting position set in SmartDashboard 
	 * 
	 * @return an enum for the starting position 
	 */
	public Position getStartingPos();
	
	/**
	 * Gets the four strategies set in SmartDashboard
	 * 
	 * @return 4 strategies, in the order of (Switch + Scale ): LL, LR, RR, RL
	 */
	public Strategy[] getStrategies();
	
	/**
	 * Gets delay before running the specified script set in SmartDashboard
	 * 
	 * @return delay in seconds
	 */
	public double getDelay();
	
	/**
	 * Directly gets the string from FMS
	 * 
	 * @return FMS data
	 */
	public String getFMS();
	
	/**
	 * Pick the script to run for Autonomous
	 * 
	 * @return A String representing the name of the script
	 */
	public String pickScript();
}
