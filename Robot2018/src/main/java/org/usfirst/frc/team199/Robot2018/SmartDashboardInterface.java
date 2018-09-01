package org.usfirst.frc.team199.Robot2018;

import edu.wpi.first.wpilibj.PIDController;

public interface SmartDashboardInterface {
	public double getConst(String key, double def);

	public void putConst(String key, double def);

	public void putData(String string, PIDController controller);

	public void putNumber(String string, double d);

	public void putBoolean(String string, boolean b);

	public String getString(String key, String def);
}
