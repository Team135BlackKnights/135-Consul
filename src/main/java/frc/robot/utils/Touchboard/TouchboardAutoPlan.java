package frc.robot.utils.Touchboard;

import java.util.List;

public class TouchboardAutoPlan {
  public int version;
  public String start; // "CENTER" etc (optional use)
  public StartPose startPose;
  public List<Step> steps;

  public static class StartPose {
    public double x;
    public double y;
    public double headingDeg;
  }

  public static class Step {
    public StepType type;

    // TRENCH only
    public TrenchChoice choice;

    public Double atSecondsRemaining;     // when to start this step
    public Double untilSecondsRemaining;  // INTAKE only (end time)
  }

  public enum StepType { TRENCH, INTAKE, CHUTE, DEPOT, HANG }
  public enum TrenchChoice { LEFT, RIGHT, BEST }
}
