package frc.robot.utils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;


import frc.robot.Constants.TuningConstants;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
public class LoggableTunedNumber {
	private static final String tableKey = "TunableNumbers";
	private final String key;
	private boolean hasDefault = false;
	private double defaultValue;
	private boolean canLogSpecific = false;
	private LoggedNetworkNumber dashboardNumber;
	private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

	/**
	 * Create a new LoggedTunableNumber
	 *
	 * @param dashboardKey Key on dashboard
	 */
	public LoggableTunedNumber(String dashboardKey) {
		this.key = tableKey + "/" + dashboardKey;
	}

	/**
	 * Create a new LoggedTunableNumber with the default value
	 *
	 * @param dashboardKey Key on dashboard
	 * @param defaultValue Default value
	 */
	public LoggableTunedNumber(String dashboardKey, double defaultValue, boolean enableValue) {
		this(dashboardKey);
		this.canLogSpecific = enableValue;
		initDefault(defaultValue, enableValue);
	}

	/**
	 * Set the default value of the number.
	 *
	 * @param defaultValue The default value
	 */
	@SuppressWarnings("unused")
	public void initDefault(double defaultValue, boolean enableValue) {
		this.defaultValue = defaultValue;
		this.canLogSpecific = enableValue;
		if (!hasDefault) {
			hasDefault = true;
			if (TuningConstants.isTuningPID && canLogSpecific && dashboardNumber == null) {
				dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
			}else if (dashboardNumber != null) {
				dashboardNumber.setDefault(defaultValue);
			}
		}
	}

	/**
	 * Get the current value, from dashboard if available and in tuning mode.
	 *
	 * @return The current value
	 */
	@SuppressWarnings("unused")
	public double get() {
		if (!hasDefault) {
			return 0.0;
		} else {
			if (TuningConstants.isTuningPID && canLogSpecific) {
				return dashboardNumber.get();
			} else {
				return defaultValue;
			}
		}
	}

	public static void ifChanged(int id, Consumer<double[]> action,
			LoggableTunedNumber... tunableNumbers) {
		if (Arrays.stream(tunableNumbers)
				.anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
			action.accept(Arrays.stream(tunableNumbers)
					.mapToDouble(LoggableTunedNumber::get).toArray());
		}
	}

	/** Runs action if any of the tunableNumbers have changed */
	public static void ifChanged(int id, Runnable action,
			LoggableTunedNumber... tunableNumbers) {
		ifChanged(id, values -> action.run(), tunableNumbers);
	}

	/**
	 * Checks whether the number has changed since our last check
	 *
	 * @param id Unique identifier for the caller to avoid conflicts when shared
	 *           between multiple objects. Recommended approach is to pass the
	 *           result of "hashCode()"
	 * @return True if the number has changed since the last time this method was
	 *         called, false otherwise.
	 */
	public boolean hasChanged(int id) {
		double currentValue = get();
		Double lastValue = lastHasChangedValues.get(id);
		if (lastValue == null || currentValue != lastValue) {
			lastHasChangedValues.put(id, currentValue);
			return true;
		}
		return false;
	}

	@SuppressWarnings("unused")
	public void changeDefault(double value){
		defaultValue = value;
		if (TuningConstants.isTuningPID && canLogSpecific && dashboardNumber == null) {
				dashboardNumber = new LoggedNetworkNumber(key, value);
			}else if (dashboardNumber != null) {
				dashboardNumber.setDefault(value);
			}
	}
}