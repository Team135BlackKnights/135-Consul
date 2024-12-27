package frc.robot.subsystems.state_space.Elevator;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface ElevatorIO {
	@AutoLog
	public static class ElevatorIOInputs {
		public double positionMeters = 0.0;
		public double velocityMetersPerSec = 0.0;
		public double elevatorTemp = 0.0;
		public double appliedVolts = 0.0;
		public double[] currentAmps = new double[] {};
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ElevatorIOInputs inputs) {}

	/** Dumb control the elevator using volts, no State Space */
	public default void setVoltage(double volts) {}

	/** Update the motor controllers to a specificed max amperage */
	public default void setCurrentLimit(int amps) {}

	/** Stop in open loop. */
	public default void stop() {}

	/**
	 * Get a list of the SelfChecking interface for all hardware in that
	 * implementation
	 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
