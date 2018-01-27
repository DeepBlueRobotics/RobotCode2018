package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem implements LiftInterface {

	private final SpeedController liftMotors = RobotMap.liftMotors;
	
	/**
	 * Set the default command for a subsystem here.
	 * */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    /**
	 * Uses (insert sensor here) to detect the distance above the ground
	 */
	public double getDistance() {
		return -1;
	}
	
	/**
	 * stops the lift
	 */
	public void stopLift() {
		liftMotors.stopMotor();
	}
	
	/**
	 * gets current motor values
	 */
	public double getLift() {
		return liftMotors.get();
	}
	
	/**
	 * goes to the bottom
	 */
	public void goToGround() {
		
	}
	
	/**
	 * goes to switch height
	 */
	public void goToSwitch() {
		
	}
	
	/**
	 * goes to scale height
	 * @param offset - the distance up or down from standard scale height
	 */
	public void goToScale(double offset) {
		
	}
	
	/**
	 * goes to bar height
	 */
	public void goToBar() {
		
	}
	
}

