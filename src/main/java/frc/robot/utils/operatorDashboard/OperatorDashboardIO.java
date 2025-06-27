package frc.robot.utils.operatorDashboard;

import org.littletonrobotics.junction.AutoLog;

public interface OperatorDashboardIO {
  @AutoLog
  class OperatorDashboardIOInputs {
    public int[] selectedLevel = new int[] {};
    public boolean[] coopState = new boolean[] {};
    public int[] topRow = new int[] {};
    public int[] midRow = new int[] {};
    public int[] hybridRow = new int[] {};
    public String targetBox = "";
  }
  default void updateInputs(OperatorDashboardIOInputs inputs){}
  default void setTopRow(int value) {
  }

  default void setMidRow(int value) {
  }

  default void setHybridRow(int value) {
  }
  default void setTargetBox(String val){}
}
