package org.usfirst.frc.team199.Robot2018.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team199.Robot2018.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
/**
 *
 */
public class Climber extends Subsystem implements ClimberInterface {

	private final SpeedController climberMotors = RobotMap.climberMotors;
	
	
	/**
	 * Set the default command for a subsystem here.
	 * */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    
   

    /**
	 * 
	 */
	public void runClimber() {
		attachToLift();
		attachToBar();
		goUp();
		
	}
	
	/**
	 * attaches the climber hook to the lift.
	 * Requires that Lift is on the ground
	 */
	public void attachToLift() {
		
	}
	
	/**
	 * winches upwards
	 */
	public void goUp() {
		
	}
	
	/**
	 * attaches hook to bar and releases it from the lift
	 */
	public void attachToBar() {
		
	}
	
	/**
	 * stops the climber
	 */
	public void stopClimber() {
		climberMotors.stopMotor();
	}

	
}

