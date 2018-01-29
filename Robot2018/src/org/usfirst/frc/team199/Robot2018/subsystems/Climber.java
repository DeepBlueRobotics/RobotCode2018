package org.usfirst.frc.team199.Robot2018.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team199.Robot2018.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class Climber extends Subsystem implements ClimberInterface {

	private final WPI_TalonSRX climberMotor = RobotMap.climberMotor;
	
	
	/**
	 * Set the default command for a subsystem here.
	 * */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    
   

    /**
	 * runs the motors
	 * 
	 */
	public void runClimber(double speed) {
		
	}
	
	/**
	 * attaches the climber hook to the lift.
	 * Requires that Lift is on the ground
	 */
	public void attachToLift() {
		
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
		climberMotor.stopMotor();
	}

	
}

