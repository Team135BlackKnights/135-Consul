package frc.robot.utils.Touchboard;

import com.google.gson.Gson;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringSubscriber;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import java.util.Optional;
import java.util.function.Supplier;

public class PosePlotterUtil {
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable datatable = inst.getTable("touchboard");

  private static String fallback = "NA";
  private static final StringSubscriber sub =
      datatable.getStringTopic("posePlotterFinalString").subscribe(fallback);

  private static final Gson GSON = new Gson();

  private static Command storedAuto = Commands.none();

  public static String getAutoString() {
    return sub.get();
  }

  public static void setFallbackAuto(String defaultValue) {
    fallback = defaultValue;
    // NOTE: subscribe() default is fixed at subscribe-time, so we enforce fallback at read-time too.
  }

  /** 200=good JSON-ish, 204=unset, 404=NA */
  public static int stringStatus() {
    String s = normalizeJsonString(sub.get());
    if (s == null || s.isEmpty() || "unset".equals(s)) return 204;
    if ("NA".equals(s)) return 404;
    return 200;
  }

  /** Parse plan if possible */
  public static Optional<TouchboardAutoPlan> tryGetPlan() {
    String norm = normalizeJsonString(sub.get());
    if (norm == null || norm.isEmpty() || "NA".equals(norm) || "unset".equals(norm)) {
      return Optional.empty();
    }
    try {
      return Optional.of(GSON.fromJson(norm, TouchboardAutoPlan.class));
    } catch (Exception e) {
      System.err.println("[PosePlotterUtil] Failed to parse JSON plan: " + e.getMessage());
      return Optional.empty();
    }
  }

  /**
   * Build + store the auto command using the supplied factory.
   * Call this when the NT string changes (typically in disabledPeriodic).
   */
  public static void calculateAuto(TouchboardAutoPlan plan, Supplier<Command> autoFactory) {
    System.err.println("Calculating Touchboard Auto.. v" + plan.version);
    storedAuto = autoFactory.get();
  }

  public static Command getAuto() {
    return storedAuto;
  }

  /**
   *  - raw JSON: {"version":...}
   *  - double-quoted JSON string: "{\"version\":...}"
   */
  public static String normalizeJsonString(String raw) {
    if (raw == null) return "NA";
    String s = raw.trim();
    if (s.isEmpty()) return "NA";

    if ("NA".equals(s) || "unset".equals(s)) return s;

    // If it's a quoted JSON string, unquote + unescape
    if (s.length() >= 2 && s.startsWith("\"") && s.endsWith("\"")) {
      s = s.substring(1, s.length() - 1);
      s = s.replace("\\\"", "\"");
      s = s.replace("\\\\", "\\");
      s = s.replace("\\n", "\n");
      s = s.replace("\\t", "\t");
      s = s.replace("\\r", "\r");
    }
    return s.trim();
  }
}
