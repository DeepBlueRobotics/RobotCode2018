package org.usfirst.frc.team199.Robot2018;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Directly copied from Robot2017 - @kevinzwang
 */

public interface DashboardInterface {

	/**
	 * Puts all desired data on SmartDashboard
	 */
	public default void displayData() {

	}

	/*
	 * Methods for displaying values with modified keys
	 */
	default void putNumber(String key, double value) {
		SmartDashboard.putNumber(getKey(key), value);
	}

	default void putBoolean(String key, boolean value) {
		SmartDashboard.putBoolean(getKey(key), value);
	}

	default void putString(String key, String value) {
		SmartDashboard.putString(getKey(key), value);
	}

	default void putSendable(String key, Sendable value) {
		SmartDashboard.putData(getKey(key), value);
	}

	/*
	 * Methods for reading numbers, without specifying the modified key
	 */
	default double getNumber(String key, double defaultValue) {
		return SmartDashboard.getNumber(getKey(key), defaultValue);
	}

	default boolean getBoolean(String key, boolean defaultValue) {
		return SmartDashboard.getBoolean(getKey(key), defaultValue);
	}

	default String getString(String key, String defaultValue) {
		return SmartDashboard.getString(getKey(key), defaultValue);
	}

	default double[] getNumArray(String key, double[] defaultValue) {
		return SmartDashboard.getNumberArray(getKey(key), defaultValue);
	}

	/**
	 * Converts the specified display key into one with its subsystem name appended
	 * as a prefix, to be compatible with the Subsystem widget on SmartDashboard for
	 * organizational purposes
	 * 
	 * @param key The name of the original key
	 * @return A modified key with prefix subsystem name
	 */
	default String getKey(String key) {
		return getClass().getSimpleName() + "/" + key;
	}
}