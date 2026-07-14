package frc.robot.subsystems.simpleMechanisms.slamElevator;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface GenericSlamElevatorIO {
  @AutoLog
  class GenericSlamElevatorIOInputs {
    public boolean motorConnected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public String name = "";
  }

  /** Update the inputs. */
  default void updateInputs(GenericSlamElevatorIOInputs inputs) {}

  /** Run slam elevator at amps */
  default void runCurrent(double amps) {}
  default void setCurrentLimit(double amps) {}
  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {
  }

  /**
   * Get a list of the SelfChecking interface for all hardware in that
   * implementation
   */
  public default List<SelfChecking> getSelfCheckingHardware() {
      return new ArrayList<SelfChecking>();
  }
}