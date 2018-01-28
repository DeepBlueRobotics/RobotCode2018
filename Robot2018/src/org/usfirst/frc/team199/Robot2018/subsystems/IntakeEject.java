package org.usfirst.frc.team199.Robot2018.subsystems;

import org.usfirst.frc.team199.Robot2018.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeEject extends Subsystem implements IntakeEjectInterface {
	
	private final WPI_TalonSRX intakeMotor = RobotMap.intakeMotor;
	
	
	
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
	public double getIntakeSpeed() {
		return intakeMotor.get();
	}
	
	/**
	 * Uses (insert sensor here) to detect 
	 * a cube in front of the robot.
	 */
	public boolean seeCube() {
		return false;
	}
	
	/**
	 * Uses (insert sensor here) to detect if 
	 * the cube is currently inside the robot
	 * 
	 */
	public boolean hasCube() {
		return false;
	}
	
	/**
	 * stops the motors
	 * 
	 */
	public void stopIntake() {
		intakeMotor.stopMotor();
	}
	
	/**
	 * Spins the rollers
	 * @param speed - positive -> rollers in, negative -> rollers out
	 */
	public void runIntake(double speed) {
		
	}
	
}

