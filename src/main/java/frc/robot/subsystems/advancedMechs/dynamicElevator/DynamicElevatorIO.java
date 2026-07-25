package frc.robot.subsystems.advancedMechs.dynamicElevator;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface DynamicElevatorIO {
    @AutoLog
    class DynamicElevatorIOInputs {
        public boolean motorConnected = true;
        public boolean followerConnected = true;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(DynamicElevatorIOInputs inputs) {
    }

    default void runOpenLoop(double output) {
    }

    default void runVolts(double volts) {
    }

    default void stop() {
    }

    /**
     * Run elevator output shaft to positionRad with additional feedforward output
     */
    default void runPosition(double positionRad, double velocity, double feedforward) {
    }

    default void setPID(double kP, double kI, double kD) {
    }

    default void setBrakeMode(boolean enabled) {
    }
    default void setCurrentLimit(int current){}

    /**
     * Get a list of the SelfChecking interface for all hardware in that
     * implementation
     */
    public default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<SelfChecking>();
    }
}
