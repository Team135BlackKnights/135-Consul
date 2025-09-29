package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsys= new ArrayList<>();

  public VirtualSubsystem() {
    subsys.add(this);
  }

  public static void periodicAll() {
    for (VirtualSubsystem subsystem : subsys) {
      subsystem.periodic();
    }
  }

  public abstract void periodic();
}
