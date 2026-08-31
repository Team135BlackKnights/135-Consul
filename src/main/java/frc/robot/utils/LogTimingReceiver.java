
package frc.robot.utils;

import org.wpilib.system.Threads;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

public class LogTimingReceiver implements LogDataReceiver {
  @Override
  public void start() {
    Threads.setCurrentThreadPriority(1);
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {}
}
