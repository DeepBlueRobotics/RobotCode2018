package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem implements LiftInterface {

	private final WPI_TalonSRX liftMotor = RobotMap.liftMotor;
	
	private Position targetPosition = Position.GROUND;
	
	/**
	 * Set the default command for a subsystem here.
	 * */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setTargetPosition(Position newPosition) {
    		targetPosition = newPosition;
    }

    /**
	 * Uses (insert sensor here) to detect the current lift position 
	 */
	public double getHeight() {
		return -1;
	}
	
	/**
	 * stops the lift
	 */
	public void stopLift() {
		liftMotor.stopMotor();
	}
	
	/**
	 * gets current motor values
	 */
	public double getLiftSpeed() {
		return liftMotor.get();
	}
	
	/**
	 * Goes to specified height
	 * @param position - ground, switch, scale, bar
	 * @param offset - distance up or down from the position
	 */
	public void goToPosition(Position position, double offset) {
		
	}
	
	
}

