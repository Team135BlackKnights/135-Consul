package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
	@AutoLog
	class ModuleIOInputs {
		public boolean driveMotorConnected = true;
		public boolean turnMotorConnected = true;
		public boolean hasCurrentControl = false;
		public boolean negateFF = false;
		public double drivePositionRads = 0.0;
		public double driveVelocityRadsPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double driveSupplyCurrentAmps = 0.0;
		public double driveTorqueCurrentAmps = 0.0;
		public double driveMotorTemp = 0.0;
		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadsPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double turnSupplyCurrentAmps = 0.0;
		public double turnTorqueCurrentAmps = 0.0;
		public double turnMotorTemp = 0.0;
		public double[] odometryDrivePositionsMeters = new double[] {};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	/** Updates the set of loggable inputs. */
	default void updateInputs(ModuleIOInputs inputs) {}

	/** Run drive motor at volts */
	default void runDriveVolts(double volts) {}

	/** Run turn motor at volts */
	default void runTurnVolts(double volts) {}

	/** Run characterization input (amps or volts) into drive motor */
	default void runCharacterization(double input) {}

	/** Run to drive velocity setpoint with feedforward */
	default void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {}

	/** Run to turn position setpoint */
	default void runTurnPositionSetpoint(double angleRads) {}

	/** Configure drive PID */
	default void setDrivePID(double kP, double kI, double kD) {}

	/** Configure turn PID */
	default void setTurnPID(double kP, double kI, double kD, double kS) {}

	/** Enable or disable brake mode on the drive motor. */
	default void setDriveBrakeMode(boolean enable) {}

	/** Enable or disable brake mode on the turn motor. */
	default void setTurnBrakeMode(boolean enable) {}

	/** Update the motor controllers to a specificed max amperage */
	default void setCurrentLimit(int amps) {}

	/** Disable output to all motors */
	default void stop() {}

	/**
	 * Get a list of the SelfChecking interface for all hardware in that
	 * implementation
	 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
