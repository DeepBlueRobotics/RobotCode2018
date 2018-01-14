package org.usfirst.frc.team199.robot.motion;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {
	static double kP, kI, kD;
	double error, integral, derivative;
	static double errorTolerance;
	public PID(double err) {
		error = err;
		integral = 0;
		derivative = 0;
		updateConstants();
	}
	
	public double write(double deltaDist) {
		error -= deltaDist;
		integral += deltaDist;
		derivative = deltaDist - derivative;
		return kP*error + kI*integral + kD*derivative;
	}
	
	public boolean isFinished() {
		return Math.abs(error) < errorTolerance;
	}
	
	public static void updateConstants() {
		kP = SmartDashboard.getNumber("kP", 0.9);
		kI = SmartDashboard.getNumber("kI", 0.01);
		kD = SmartDashboard.getNumber("kD", 0);
		errorTolerance = SmartDashboard.getNumber("kerrorTolerance", 1);
	}
}
