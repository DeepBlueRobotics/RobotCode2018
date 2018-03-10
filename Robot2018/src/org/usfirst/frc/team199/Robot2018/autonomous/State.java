package org.usfirst.frc.team199.Robot2018.autonomous;

public class State {
	private double currX;
	private double currY;
	private double currRotation;
	private double centerOfRot;

	public State(double x, double y, double rot, double centerOfRot) {
		currX = x;
		currY = y;
		currRotation = rot;
		this.centerOfRot = centerOfRot;
	}

	/*
	 * All of these are getters and setters for the robot's position and orientation
	 */
	public double getX() {
		return currX;
	}

	public double getY() {
		return currY;
	}

	public double getRot() {
		return currRotation;
	}

	public void setX(double x) {
		currX = x;
	}

	public void setY(double y) {
		currY = y;
	}

	public void setRot(double rot) {
		currRotation = rot;
	}

	public void changeX(double x) {
		currX += x;
	}

	public void changeY(double y) {
		currY += y;
	}

	public void changeRot(double rot) {
		currRotation += rot;
	}
	
	public double getCenterOfRot() {
		return centerOfRot;
	}
}
