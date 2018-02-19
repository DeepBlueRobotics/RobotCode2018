package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.OI;
import org.usfirst.frc.team199.Robot2018.RobotMap;
import org.usfirst.frc.team199.Robot2018.commands.AutoLift;
import org.usfirst.frc.team199.Robot2018.subsystems.LiftInterface.Position;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem implements LiftInterface {

	private final SpeedControllerGroup liftMotors = RobotMap.liftMotors;
	private final Encoder liftEnc = RobotMap.liftEnc;
	private Position currPosition = Position.GROUND;
	
	/**
	 * Set the default command for a subsystem here.
	 * */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	int ang = OI.manipulator.getPOV();
    	Command com = null;
    	if(ang == 0)
    		com = new AutoLift(Position.GROUND, this);
    	else if(ang == 90)
    		com = new AutoLift(Position.SWITCH, this);
    	else if(ang == 180)
    		com = new AutoLift(Position.SWITCH, this);
    	else if(ang == 270)
    		com = new AutoLift(Position.SWITCH, this);
    	if(com != null)
    		setDefaultCommand(com);
    }
    
    /**
	 * Sets the current position in the lift subsystem
	 * @param newPosition - the new position meant to be set
	 */
    public void setCurrPosition(Position newPosition) {
    		currPosition = newPosition;
    }

    /**
	 * Uses (insert sensor here) to detect the current lift position 
	 */
	public double getHeight() {
		return liftEnc.getDistance() * 3;
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
	public double getLiftSpeed() {
		return liftMotors.get();
	}
	
	/**
	 * Runs lift motors at specified speed
	 * @param speed - desired speed to run at
	 */
	public void runMotor(double output) {
		liftMotors.set(output);
	}
	
	/**
	 * Returns the position the lift is currently at
	 * @return pos - current position
	 */
	public Position getCurrPos() {
		return currPosition;
	}
	/**
	 * Resets the encoder
	 */
	public void resetEnc() {
		liftEnc.reset();
	}
	
}

