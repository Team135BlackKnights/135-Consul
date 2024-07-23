// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface TankIO {
	@AutoLog
	public static class TankIOInputs {
		public double leftPositionRad = 0.0;
		public double leftVelocityRadPerSec = 0.0;
		public double leftAppliedVolts = 0.0;
		public double[] leftCurrentAmps = new double[] {};
		public double frontLeftDriveTemp = 0.0;
		public double backLeftDriveTemp = 0.0;
		public double rightPositionRad = 0.0;
		public double rightVelocityRadPerSec = 0.0;
		public double rightAppliedVolts = 0.0;
		public double[] rightCurrentAmps = new double[] {};
		public double frontRightDriveTemp = 0.0;
		public double backRightDriveTemp = 0.0;
		public Rotation2d gyroYaw = new Rotation2d();
		public boolean gyroConnected = true;
		public boolean collisionDetected = false;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(TankIOInputs inputs) {}

	/** Run open loop at the specified voltage. */
	public default void setVoltage(double leftVolts, double rightVolts) {}
	/** Update the motor controllers to a specificed max amperage */
	public default void setCurrentLimit(int amps){}
	/** Resets the Gyro */
	public default void reset() {}

	/** Run closed loop at the specified velocity. */
	public default void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {}

	/**
	 * Get a list of the SelfChecking interface for all hardware in that
	 * implementation
	 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
