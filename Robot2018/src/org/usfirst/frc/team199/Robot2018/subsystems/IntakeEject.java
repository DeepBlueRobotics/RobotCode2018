package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeEject extends Subsystem implements IntakeEjectInterface {
	
	private final SpeedController intakeRollers = RobotMap.intakeMotors;
	
	
	
	/**
	 * Set the default command for a subsystem here.
	 * */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    /**
	 * returns current motor value
	 */
	public double getIntake() {
		return intakeMotors.get();
	}
	
	/**
	 * Uses (insert sensor here) to detect 
	 * a cube in front of the robot.
	 */
	public boolean detectCube() {
		
	}
	
	/**
	 * Uses (insert sensor here) to detect if 
	 * the cube is currently inside the robot
	 * 
	 */
	public boolean hasCube() {
		
	}
	
	/**
	 * stops the motors
	 * 
	 */
	public boolean stopIntake() {
		intakeRollers.stop();
	}
	
	/**
	 * Spins the rollers
	 * @param speed - positive -> rollers in, negative -> rollers out
	 */
	public void runIntake(double speed) {
		
	}
	
}

