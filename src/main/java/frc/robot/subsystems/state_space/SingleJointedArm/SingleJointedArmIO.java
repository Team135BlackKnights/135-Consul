package frc.robot.subsystems.state_space.SingleJointedArm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface SingleJointedArmIO {
	@AutoLog
	public static class SingleJointedArmIOInputs {
		public double positionRad = 0.0;
		public double velocityRadPerSec = 0.0;
		public double armTemp = 0.0;
		public double appliedVolts = 0.0;
		public double[] currentAmps = new double[] {};
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(SingleJointedArmIOInputs inputs) {}

	/** Dumb control the Arm using volts, no State Space */
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
