package frc.robot.utils;
import org.wpilib.system.Timer;

public final class AutoTime {
  private AutoTime() {}

  public static final double AUTO_LENGTH_SEC = 20.0;

  private static final Timer autoTimer = new Timer();
  private static boolean started = false;

  public static void startAuto() {
    autoTimer.stop();
    autoTimer.reset();
    autoTimer.start();
    started = true;
  }

  public static void stop() {
    autoTimer.stop();
    started = false;
  }

  public static boolean isRunning() {
    return started;
  }

  public static double elapsed() {
    return started ? autoTimer.get() : 0.0;
  }

  public static double remaining() {
    double r = AUTO_LENGTH_SEC - elapsed();
    return Math.max(0.0, r);
  }
}
