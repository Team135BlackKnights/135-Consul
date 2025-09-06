
package frc.robot.utils;

import edu.wpi.first.wpilibj.Threads;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

public class LogTimingReceiver implements LogDataReceiver {
  @Override
  public void start() {
    Threads.setCurrentThreadPriority(true, 1);
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {}
}