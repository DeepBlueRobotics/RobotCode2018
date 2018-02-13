package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.Position;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem implements LiftInterface {

	private final WPI_TalonSRX liftMotor = RobotMap.liftMotor;
	private final Encoder liftEnc = RobotMap.liftEnc;
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
	/**
	 * Runs lift motor at specified speed
	 * @param speed - desired speed to run at
	 */
	public void runMotor(double output) {
		liftMotor.set(output);
	}
	
	/**
	 * Returns the position the lift is currently at
	 * @return pos - current position
	 */
	public Position getCurrPos() {
		return targetPosition;
	}
	/**
	 * Resets the encoder
	 */
	public void resetEnc() {
		liftEnc.reset();
	}
	
}

