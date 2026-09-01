package frc.robot.utils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import frc.robot.Constants.TuningConstants;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Class for a tunable boolean. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
@SuppressWarnings("unused") // Some methods will report "dead" code, since finals exist which are designed to be modified.
public class LoggableTunedBoolean implements BooleanSupplier {
  private static final String tableKey = "TunableNumbers";
  private final String key;
  private boolean hasDefault = false;
  private boolean defaultValue;
  private boolean canLogSpecific = false;
  private LoggedNetworkBoolean dashboardBoolean;
  private Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggableTunedBoolean
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggableTunedBoolean(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggableTunedBoolean with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   * @param enableValue Whether to enable tuning for this value
   */
  public LoggableTunedBoolean(String dashboardKey, boolean defaultValue, boolean enableValue) {
    this(dashboardKey);
    this.canLogSpecific = enableValue;
    initDefault(defaultValue, enableValue);
  }

  /**
   * Set the default value of the boolean.
   *
   * @param defaultValue The default value
   * @param enableValue Whether to enable tuning for this value
   */
  public void initDefault(boolean defaultValue, boolean enableValue) {
    this.defaultValue = defaultValue;
    this.canLogSpecific = enableValue;
    if (!hasDefault) {
      hasDefault = true;
      if (TuningConstants.isTuningPID && canLogSpecific && dashboardBoolean == null) {
        dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
      } else if (dashboardBoolean != null) {
        dashboardBoolean.setDefault(defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public boolean get() {
    if (!hasDefault) {
      return false;
    } else {
      if (TuningConstants.isTuningPID && canLogSpecific) {
        return dashboardBoolean.get();
      } else {
        return defaultValue;
      }
    }
  }

  /** Runs action if any of the tunableBooleans have changed */
  public static void ifChanged(int id, Consumer<boolean[]> action,
      LoggableTunedBoolean... tunableBooleans) {
    if (Arrays.stream(tunableBooleans)
        .anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id))) {
      boolean[] values = new boolean[tunableBooleans.length];
      for (int i = 0; i < tunableBooleans.length; i++) {
        values[i] = tunableBooleans[i].get();
      }
      action.accept(values);
    }
  }

  /** Runs action if any of the tunableBooleans have changed */
  public static void ifChanged(int id, Runnable action,
      LoggableTunedBoolean... tunableBooleans) {
    ifChanged(id, values -> action.run(), tunableBooleans);
  }

  /**
   * Checks whether the boolean has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared
   *           between multiple objects. Recommended approach is to pass the
   *           result of "hashCode()"
   * @return True if the boolean has changed since the last time this method was
   *         called, false otherwise.
   */
  public boolean hasChanged(int id) {
    boolean currentValue = get();
    Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /**
   * Programmatically set the value (e.g. reset to false for a one-shot flag).
   * Updates both the local default and the dashboard entry so the
   * next call to {@link #get()} returns the new value.
   *
   * @param value The value to set.
   */
  public void set(boolean value) {
    defaultValue = value;
    if (dashboardBoolean != null) {
      dashboardBoolean.set(value);
    }
  }

  public void changeDefault(boolean value) {
    defaultValue = value;
    if (TuningConstants.isTuningPID && canLogSpecific && dashboardBoolean == null) {
      dashboardBoolean = new LoggedNetworkBoolean(key, value);
    } else if (dashboardBoolean != null) {
      dashboardBoolean.setDefault(value);
    }
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
