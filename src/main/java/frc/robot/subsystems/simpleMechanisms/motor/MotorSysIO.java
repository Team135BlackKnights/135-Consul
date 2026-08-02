package frc.robot.subsystems.simpleMechanisms.motor;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface MotorSysIO {
  @AutoLog
  abstract class MotorSysIOInputs {
    public boolean connected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public String name = "";
  }

  default void updateInputs(MotorSysIOInputs inputs) {
  }

  default void setPosition(double angularPositionRads) {
  }

  default void setCurrentLimit(double amps) {
  }

  /** Stop feeder */
  default void stop() {
  }

  /**
   * Get a list of the SelfChecking interface for all hardware in that
   * implementation
   */
  public default List<SelfChecking> getSelfCheckingHardware() {
    return new ArrayList<SelfChecking>();
  }
}